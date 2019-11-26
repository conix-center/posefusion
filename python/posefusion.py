import cv2
import numpy as np
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import json
import paho.mqtt.client as mqtt
import time
import pandas as pd
import ipdb
from datetime import datetime

import calibration

# Global constants

# MQTT
SERVER_ADDR     = "oz.andrew.cmu.edu"
TOPIC_3D        = "/posefusion/skeleton"
TOPIC_POSE      = "/lambda/+/pose"
TOPIC_CAMERA    = "/posefusion/camera/"
CLIENT_ID       = "posefusion"

# Paths
MATRICES_PATH   = "matrices.xml"
PROJS_PATH      = "../../results/projections.npz"
INTRINSICS_PATH = "calib1.npz"

# Global parameters
# Configuation
MAX_NUM_PEOPLE  = 20
NUM_CAMERAS     = 3
REF_CAM         = 0
CONF_SCORE      = 0.6

# Calibration
LOAD_PROJS      = True
MIN_BODY_CALIB  = 1000

# Reconstruction
ERR_THRESHOLD   = 30000
FRAME_AVERAGING = 1
MIN_BODY_POINTS = 50

# Globals
running = False
received_data = [False, False, False]
calib_done = False

'''
read_matrices: Read projection matrices from the specified file path in a numpy array of 
               shape (NUM_CAMERAS, 3, 4).
    Input:  F, fundamental matrix
    Output: projection_matrices, projection matrices (NUM_CAMERAS, 3, 4)
'''
def read_matrices(file_path):
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    numCameras = int(cv_file.getNode("NumCameras").mat())
    projection_matrices = np.empty((numCameras, 3, 4))
    for camera in range(0, numCameras):
        matrix = cv_file.getNode("Projection" + str(camera)).mat()
        projection_matrices[camera] = matrix
    return projection_matrices

'''
repro_error: compute the reprojection error of a computed 3D point.
    Input:  pts4D, points in 3D in homogeneous coordinates (4th point is 1)
            proj0, projection matrix of first camera used for triangulation
            proj1, projection matrix of second camera used for triangulation
            pt1, point of first camera
            pt2, point of second camera
    Output: total, reprojection error
            sum_per_part, error per body part
'''
def repro_error(pts4D, proj0, proj1, pt1, pt2):
    # Obtain estimation of pt1
    pt1_est = np.dot(proj0,pts4D.T).T
    
    # Obtain 2D coordinates normalized (divide by z, I think)
    pt1_est = pt1_est[:, :2]/np.repeat(pt1_est[:, 2], 2).reshape(-1, 2)
    
    # Obtain error for each point 
    error = np.sqrt(np.sum((pt1_est-pt1.T)**2, axis=1))
    
    # Obtain estimation of pt2
    pt2_est = np.dot(proj1,pts4D.T).T
    
    # Obtain 3D coordinates normalized (divide by z, I think)
    pt2_est = pt2_est[:, :2]/np.repeat(pt2_est[:, 2], 2).reshape(-1, 2)
    
    # Obtain error for each point 
    error2 = np.sqrt(np.sum((pt2_est-pt2.T)**2, axis=1))
    
    sum_per_part = (error + error2) / 2
    total = np.sum(sum_per_part)
    
    return total, sum_per_part

'''
distance_points: compute the distance between two points.
    Input:  p1, first point coordinate (numpy array)
            p2, second point coordinate (numpy array)
    Output: dist, distance between the two points provided
'''
def distance_points(p1, p2):
    squared_dist = np.sum((p2-p1)**2, axis = 0)
    dist = np.sqrt(squared_dist)
    return dist

# INTERPOLATE PARTS OF THE BODY IF POSSIBLE!!

'''
fusion_body: merge two bodies. If the body part of the new body is non zero then the fusioned body
             used two coordinates, otherwise it uses the previous body coordinates for that part.
    Input:  prev_body, body points of previous body
            new_body, body points of new body
    Output: fusioned_body, body points resulting from the mix of previous and new body
'''
def fusion_body(prev_body, new_body):
    fusioned_body = np.empty((25,3))
    for i in range(prev_body.shape[0]):
        if (np.count_nonzero(prev_body[i]) == 0):
            fusioned_body[i] = new_body[i]
        else:
            fusioned_body[i] = prev_body[i]
    return fusioned_body

'''
check_body_length: given a body in 3D it applies several filters to ensure that the overall shape
                   of the body is valid.
    Input:  body, body points in 3D
'''
def check_body_length(body):
    # Arms
    r_upper_arm = distance_points(body[2], body[3])
    r_lower_arm = distance_points(body[3], body[4])
    r_arm = r_upper_arm + r_lower_arm
    l_upper_arm = distance_points(body[5], body[6])
    l_lower_arm = distance_points(body[6], body[7])
    l_arm = l_upper_arm + l_lower_arm

    # Legs
    r_upper_leg = distance_points(body[9], body[10])
    r_lower_leg = distance_points(body[10], body[11])
    r_leg = r_upper_leg + r_lower_leg
    l_upper_leg = distance_points(body[12], body[13])
    l_lower_leg = distance_points(body[13], body[14])
    l_leg = l_upper_leg + l_lower_leg
    
    # Feet
    r_foot = distance_points(body[22], body[24])
    r_foot2 = distance_points(body[11], body[23])
    l_foot = distance_points(body[19], body[21])
    l_foot2 = distance_points(body[14], body[20])

    # Head 
    neck_head = distance_points(body[0], body[1])
    l_head = distance_points(body[17], body[15])
    r_head = distance_points(body[18], body[16])
    dist_neck_l_eye = distance_points(body[0], body[15])
    dist_neck_r_eye = distance_points(body[0], body[16])
    dist_eyes = distance_points(body[15], body[16])
    dist_ears = distance_points(body[17], body[18])
        
    if (l_head > 0.5):
        body[17] = body[15] = 0

    if (r_head > 0.5):
        body[18] = body[16] = 0

    if (dist_neck_l_eye > 0.5):
        body[15] = 0

    if (dist_neck_r_eye > 0.5):
        body[16] = 0

    if (neck_head > 0.5):
        body[0] = 0
        
     # Check right arm lengths
    if ((r_arm > 1) or (r_arm <= 0)):
        body[2] = body[3] = body[4] = 0
    
    # Check arm lengths
    if ((l_arm > 1) or (l_arm <= 0)):
        body[5] = body[6] = body[7] = 0

    # Check left leg lengths
    if ((l_upper_leg > 1) or (l_upper_leg <= 0)):
        body[9] = body[10] = 0
    
    if ((l_lower_leg > 1) or (l_lower_leg <= 0)):
        body[10] = body[11] = 0
             
    # Check right leg lengths
    if ((r_upper_leg > 1) or (r_upper_leg <= 0)):
        body[12] = body[13] = 0
    if ((r_lower_leg > 1) or (r_lower_leg <= 0)):
        body[13] = body[14] = 0

    # Check foot length
    if (r_foot > 0.3) or (r_foot <= 0):
        body[22] = body[24] = body[11] = body[23] = 0
        
    if (l_foot > 0.3) or (l_foot <= 0):
        body[14] = body[19] = body[20] = body[21] = 0

    if (r_foot2 > 0.3) or (r_foot2 <= 0):
        body[22] = body[24] = body[11] = body[23] = 0
        
    if (l_foot2 > 0.3) or (l_foot2 <= 0):
        body[14] = body[19] = body[20] = body[21] = 0

    return True

'''
filter_body: apply different filters to the body
    Input:  body, body points in 3D
'''
def filter_body(body):
    # Body length
    check_body_length(body)
    
    # Max greater than 10
    body[body > 10] = 0
    
    # Min less than -10
    body[body < -10] = 0

    
    if (not np.all(np.isfinite(body))):
        print("VALUE IS NOT FINITE")
        return False
    
    return True
    
'''
triangulateTwoBodies: obtain 3D reconstruction of body given body coordinates from two different 
                      cameras.
    Input:  camera_1, projection matrix of first camera
            camera_1, projection matrix of second camera
            pt1, 2D coordinates of body of first camera
            pt2, 2D coordinates of body of second camera
    Output: error, overall reprojection error (scalar)
            error_mat, reprojection error per body part
            pts3D, 3D coordinates obtained from triangulation
'''
def triangulateTwoBodies(camera_1, camera_2, pt1, pt2):
    # Transform into appropriate format for OpenCV
    pt1 = np.vstack((pt1[:,0], pt1[:,1]))
    pt2 = np.vstack((pt2[:,0], pt2[:,1]))

    # For each
    pts4D = cv2.triangulatePoints(camera_1, camera_2, pt1, pt2).T



    # Convert from homogeneous coordinates to 3D
    pts3D = pts4D[:, :3]/np.repeat(pts4D[:, 3], 3).reshape(-1, 3)


    neck_hip = distance_points(pts3D[1], pts3D[8])
    # print(neck_hip)
    # Normalize
    pts3D = pts3D[:, :3] / (neck_hip*1.2)
    # print(pts3D[11,1])

    # Find mean y position foot
    mean_foot = np.mean(pts3D[[11, 14, 19, 20, 21, 22, 23, 24],1])
    # print(pts3D[[11, 14, 19, 20, 21, 22, 23, 24],1])
    # print(mean_foot)

    # pts3D[:,1] -= pts3D[11,1]
    # Put person on ground
    pts3D[:,1] -= mean_foot

    # Normalize 4D
    pts4D = pts4D[:, :4]/np.repeat(pts4D[:, 3], 4).reshape(-1, 4)

    # Get error
    error, error_mat = repro_error(pts4D, camera_1, camera_2, pt1, pt2)
    return error, error_mat, pts3D

'''
convertTo3DPoints: given a set of 2D coordinates cameras, find best 3D reconstruction.
    Input:  ref_cam, reference camera used to perform triangulation
            number_cameras, number of cameras
    Output: saved3DCoordinates, 3D coordinates of the valid bodies found
            num_body_ok, number of bodies returned
            list_cam_chosen, list of the cameras chosen for triangulation
'''
def convertTo3DPoints(ref_cam, number_cameras):
    cam_chosen = 0
    body_num_chosen = 0
    num_body_ok = 0
             
    list_cams = list(range(number_cameras))
    list_cams.remove(ref_cam)

    list_cam_chosen = []
    
    # Get number of people: assume visibile in three cameras
    num_bodies_ref_cam = get_num_body_camera(ref_cam)
    
    # Hold the 3D coordinates after triangulation
    saved3DCoordinates = np.empty((25, 3, num_bodies_ref_cam))
    
    # Go through each body in camera 0
    for body_num_cam_ref in range(num_bodies_ref_cam):
        error = 10000000000
        send = False
        first_points = dataPoints[ref_cam, :, :, body_num_cam_ref]
        for camera_num in list_cams:
            for body_num in range(get_num_body_camera(camera_num)):
                # Triangulate points
                second_points = dataPoints[camera_num, :, :, body_num]
                new_error, error_mat, pts3D = triangulateTwoBodies(projs[ref_cam], projs[camera_num], first_points, second_points)
                # print(camera_num, body_num_cam_ref, body_num, new_error)
                
                # If triangulation error is less than the previous and the max threshold, send it
                if ((new_error < error) and (new_error < ERR_THRESHOLD)):
                    send = True
                    saved3DCoordinates[:,:,num_body_ok] = pts3D
                    cam_chosen = camera_num
                    body_num_chosen = body_num
                    error = new_error

        if (send):
            # print("[{}]: cam{}, body{}, error:{}".format(body_num_cam_ref, cam_chosen, body_num_chosen, error))
            list_cam_chosen.append(cam_chosen)
            num_body_ok += 1
            
    # Fill data Points with 0
    dataPoints.fill(0)
        
    return saved3DCoordinates, num_body_ok, list_cam_chosen

'''
get_num_body_camera: helper function to find number of bodies detected by a camera
    Input:  camera_num, camera of interest
    Output: body_count, number of bodies found by that camera in that frame
'''
def get_num_body_camera(camera_num):
    body_count = 0
    for i in range(dataPoints.shape[3]):
        if (np.any(dataPoints[camera_num,:,:,i]) == True):
            body_count = body_count + 1
    return body_count

'''
find_new_mapping: find the new mapping of bodies to points
    Input:  prev, set of previous bodies
            new, set of new bodies
            numBodies, number of bodies found
            prev_mapping, previous mapping
    Output: mapping, new mapping
            prev_updated, replacement for the previous body array
'''
def find_new_mapping(prev, new, numBodies, prev_mapping):
    # Initialize the new mapping from the previous one
    mapping = prev_mapping.copy()

    prev_updated = np.empty((25, 3, MAX_NUM_PEOPLE))

    # Create list of bodies
    body_list = list(range(numBodies))

    # For each of the previous body detected up to numBodies
    for person in range(numBodies):
        error = 666666666
        min_b = 0
        prev_ = prev[:,:,prev_mapping[person]]

        # Check which of the new body is the closest to the previous one
        for body_num in body_list:
            new_ = new[:,:,body_num]
            # new_error = np.linalg.norm(new_-prev_)
            new_error = distance_points(new_[0], prev_[0])
            # print(prev_[:5], new_[:5])
            # print(np.count_nonzero(prev_))
            # print("person{}, prev_body:{}, current b:{}, error:{}".format(person, prev_mapping[person], body_num, round(new_error,2)))
            # if (new_error < error and new_error != 0) or (error == 666666666):
            if (new_error < error) or (error == 666666666):
                mapping[person] = body_num
                error = new_error
                min_b = body_num

        body_list.remove(min_b)
        # prev_updated[:,:,min_b] = fusion_body(prev_, new[:,:,min_b])
        prev_updated[:,:,min_b] = new[:,:,min_b]

    return mapping, prev_updated

######################### MQTT Functions #########################
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(TOPIC_POSE)


def on_message(client, userdata, msg):
    # Obtain lambda ID from message
    lambda_id = int(msg.topic[8])
    
    # Convert message into string
    rcv_msg = str(msg.payload.decode("utf-8"))

    # Frame containing body coordinates
    if (len(rcv_msg) > 50):

        people_list = rcv_msg.split(",")

        for body in people_list[:-1]:
            person_id, arr, ts, score = parse_body_from_mqtt(body)

            global calib_done

            if (calib_done == False):
                # Add to pts of whatever lambda
                if (score > CONF_SCORE):
                    print(len(pts_calib[0]))
                    pts_calib[lambda_id-1].append(arr)
                    time_calib[lambda_id-1].append(ts)

            if (len(pts_calib[0]) > MIN_BODY_CALIB):
                calib_done = True

            # Store data in appropriate array if the calibration is done and confidence score large enough
            if (calib_done == True):
                if (score > CONF_SCORE):
                    dataPoints[lambda_id-1, :, :, int(person_id)] = arr
    # Synchronization frame
    else:
        dataPoints[lambda_id-1].fill(0)

    # print("{}, {}, {}".format(ts, lambda_id, person_id))
    received_data[lambda_id - 1] = True


'''
parse_body_from_mqtt: obtain the id of the person, 2D points, timestamp and confidence score
                      from part of MQTT message for a person
    Input:  message, string of the received message
    Output: person_id, id of the person from OpenPose
            body_arr, array of 2D points (25,2)
            ts, timestamp of the frame
            conf_score, confidence score obtained from OpenPose for that person
'''
def parse_body_from_mqtt(message):
    l = message.split()
    body_arr = np.array(l[3:], dtype=float).reshape(25,2)
    conf_score = float(l[2])
    person_id = l[1]
    ts = l[0]
    return person_id, body_arr, ts, conf_score

'''
publish_person: given the ID of a person and its 3D coordinates, prepare an MQTT message.
    Input:  coord_3D, 3D coordinates to be published
            person_id, id of the person
    Output: message, string of the message to be sent
'''
# Parse array of 3D coordinates of a person to string to be sent to MQTT
def publish_person(coord_3D, person_id):
    message = json.dumps(coord_3D.flatten().tolist())
    message = message.strip('[]')
    message = 'Person' + str(person_id) + ',' + message + ",on"
    return message

'''
publish_camera: generate message to be sent through MQTT of the [tx, ty, tz] vector of projection matrix
    Input:  projection, projection matrix (3, 4)
    Output: message, string of the message to be sent
'''
def publish_camera(projection):
    # zero_coord = np.array([1, 1, 1, 1])    
    zero_coord = np.array([0, 0, 0, 1])      
    point = np.dot(projection, zero_coord)
    message = json.dumps(point.flatten().tolist())
    message = message.strip('[]')
    return message

######################### END #########################

    
if __name__ == '__main__':
    pts_calib = {new_list: [] for new_list in range(NUM_CAMERAS)} 
    time_calib = {new_list: [] for new_list in range(NUM_CAMERAS)} 

    if (LOAD_PROJS == True):
        calib_done = True

    ################## MQTT Init ################## 
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    ################## MQTT Connect ################## 
    client.connect(SERVER_ADDR, port=1883, keepalive=60)

    ################## CALIBRATION ################## 
    while (calib_done == False):
        client.loop(0.1)
    print("Collect data finished")

    np.savez("../../results/pts_calib.npz", pts_calib0=pts_calib[0],pts_calib1=pts_calib[1],pts_calib2=pts_calib[2], date=int(time.time()))
    np.savez("../../results/time_calib.npz", time_calib0=time_calib[0],time_calib1=time_calib[1],time_calib2=time_calib[2], date=int(time.time()))

    # Create empty numpy array to hold projection matrices
    projs = np.empty((NUM_CAMERAS, 3, 4))
    m_matrices = np.empty((NUM_CAMERAS, 3, 4))

    # Default intrinsic matrices
    K1 = K2 = np.load(INTRINSICS_PATH)["mtx"]     

    # Load projection matrices
    if (LOAD_PROJS == True):
        prev_projs = np.load(PROJS_PATH)
        dt_object = datetime.fromtimestamp(prev_projs["date"])
        date_time = dt_object.strftime("%m/%d/%Y, %H:%M:%S")
        projs = prev_projs["projs"]
        m_matrices = prev_projs["m_matrices"]
        print("Loaded projections for {} cameras from [{}]".format(prev_projs["num_cameras"], date_time))
    # Compute new projection matrices
    else:
        # Iterate through each pair of camera
        for camera_num in range(NUM_CAMERAS-1):
            print("Camera{}: {} bodies collected".format(camera_num+1, len(pts_calib[camera_num+1])))
            projs[0], projs[camera_num + 1], m_matrices[0], m_matrices[camera_num + 1] = calibration.get_projs_matrices(pts_calib[0], pts_calib[camera_num + 1], K1, K2)
        
        # projs[1], projs[0], m_matrices[1], m_matrices[0] = calibration.get_projs_matrices(pts_calib[1], pts_calib[0], K1, K2)
        # projs[1], projs[2], m_matrices[1], m_matrices[2] = calibration.get_projs_matrices(pts_calib[1], pts_calib[2], K1, K2)
        # Save resulting projection matrices
        np.savez(PROJS_PATH, num_cameras = NUM_CAMERAS, K1=K1, K2=K2, projs=projs, m_matrices = m_matrices,date=int(time.time()))

    # Read matrices.xml
    # projs = read_matrices(MATRICES_PATH)

    ################## INITIALIZATION BEFORE ALGORITHM ################## 
    # Hold people coordinates
    global dataPoints
    dataPoints = np.empty((NUM_CAMERAS, 25, 2, MAX_NUM_PEOPLE))
    prev3DPoints = np.empty((25, 3, MAX_NUM_PEOPLE))

    # Hold current frame number for averaging
    frame = 0

    # Initialize mapping function
    id_mapping = {}
    for i in range(MAX_NUM_PEOPLE):
        id_mapping[i] = i


    # print(0, m_matrices[0, :, -1])
    # print(1, m_matrices[1, :, -1])
    # print(2, m_matrices[2, :, -1])

    # print(0, projs[0])
    # print(1, projs[1])
    # print(2, projs[2])

    # print(1, m_matrices[1, :, -1])
    # print(2, m_matrices[2, :, -1])

    # MQTT loop
    while True:
        # Triangulate
        if (received_data[0] and received_data[1] and received_data[2]):
            received_data[0] = received_data[1] = received_data[2] = False
            if (not running):
                # Make sure we don't run that algorithm conccurently
                running = True

                # Publish camera coordinates to be displayed in ARENA
                for i in range(NUM_CAMERAS):
                    client.publish(TOPIC_CAMERA + str(i), publish_camera(m_matrices[i])) 

                # 1. Obtain 3D coordinates of people using triangulation
                points3D, numBodies, list_cam_chosen = convertTo3DPoints(REF_CAM, NUM_CAMERAS)
                if (len(list_cam_chosen)):
                        print(list_cam_chosen)

                # 2. Filter new body that arrived
                for body_num in range(numBodies):
                    filter_body(points3D[:,:,body_num])

                # 3. Find new mapping of people id
                # id_mapping, prev3DPoints = find_new_mapping(prev3DPoints, points3D, numBodies, id_mapping)
                id_mapping, prev = find_new_mapping(prev3DPoints, points3D, numBodies, id_mapping)
                
                # Publish each body separately
                for person in range(numBodies):                
                    # Fusion prev body and new body
                    # !!! Might need to fusion in find_new_mapping and take into account id_mapping
                    # prev3DPoints[:,:,person] = fusion_body(prev3DPoints[:,:,person], points3D[:,:,person])
                    # prev3DPoints[:,:,person] = points3D[:,:,person]

                    # Done averaging, publish it
                    if (frame == FRAME_AVERAGING):
                        # Filter body (after fusion?)
                        filter_body(points3D[:,:,id_mapping[person]])

                        # Count number of zero elements in the points3D
                        nz = np.count_nonzero(points3D[:,:,id_mapping[person]])
                        if (nz > MIN_BODY_POINTS):
                            # print("Person " + str(person) +  "[" +str(id_mapping[person]) + "]: non-zeros parts = " + str(nz) + "---------SENDING")
                            print("[{}] Person{}[{}]: NZ parts = {} ------------- SENDING".format(time.time(), person, id_mapping[person], nz))
                            client.publish(TOPIC_3D, publish_person(points3D[:,:,id_mapping[person]], person)) 
                            # print(points3D[:,:,id_mapping[person]])
                        else:
                            # print("Person " + str(person) +  "[" +str(id_mapping[person]) + "]: non-zeros parts = " + str(nz))
                            print("Person{}[{}]: NZ parts = {}".format(person, id_mapping[person], nz))
                        # prev3DPoints[:,:,person].fill(0)


                # Send zeros for the rest of the bodies
                if (frame == FRAME_AVERAGING):
                    for b in range(numBodies, 6):
                        emp = np.zeros((25, 3))
                        # client.publish(TOPIC_3D, publish_person(emp, b))
                    frame = 0
                else:
                    frame +=1

                # Replace previous points with new points and 0 for the rest
                for person in range(numBodies):
                    prev3DPoints[:,:,person] = prev[:,:,person]
                    prev3DPoints[:,:,person] = points3D[:,:,person]
                for person in range(numBodies, MAX_NUM_PEOPLE):
                    prev3DPoints[:,:,person].fill(0)

                # Done with this funtion
                running = False
        # End received frames from three lambdas

        # Get new messages
        client.loop(0.01)