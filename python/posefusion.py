import cv2
import numpy as np
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import json
import paho.mqtt.client as mqtt
import time
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
PROJS_PATH      = "projections.npz"
AFFINE_PATH      = "affine.npz"
INTRINSICS_PATH = "calib1.npz"

# Global parameters
# Configuation
MAX_NUM_PEOPLE  = 20     # Maximum number of people supported
NUM_CAMERAS     = 2      # (2 stereos = 2 * 2 cameras = 4)
REF_CAM         = 0      # Stereo pair of referencce (0 is for camera0-camera1)
CONF_SCORE      = 0.6    # Minimum confidence score for a body required to be considered valid (from OpenPose wrapper)
USE_STEREO_3D   = False  # If set to true, s3D pts are obtained from stereo (math), otherwise uses projection matrices

# Calibration
RUN_CALIBRATION = False  # If set to True the autocalibration will be ran, otherwise PROJS_PATH and AFFINE_PATH will be used
MIN_BODY_CALIB  = 1000   # Number of skeletons to collect during autocalibration to obtain projection matrices
MIN_BODY_AFFINE = 250     # Number of 3D skeletons required to get affine transformation

# Reconstruction
ERR_THRESHOLD   = 2000
FRAME_AVERAGING = 1     # Number of frames to average
MIN_BODY_POINTS = 50    # Minimun number of body points that are non-zero to sent skeleton to ARENA

# Globals
received_data = [False] * NUM_CAMERAS    # True means that data have been received for that lambda
num_skels_before_transform = 0           # Keep track of how nmany bodies found before finding the transformation R, t
R = 0                                    # Rotation matrix for second stereo pair
t = 0                                    # Translation vector for second stereo pair
scale =  1                               # Scale scalar for second stereo pair
y_offset = 0                             # Distance body from reconstruction to ground
# Store data points from OpenPose
dataPoints = np.empty((NUM_CAMERAS, 25, 2, MAX_NUM_PEOPLE))
# Store 3D skels to obtain affine transform, 2 stereo cameras, 75 bodies of 25x3
pts_affine = np.zeros((2, MIN_BODY_AFFINE, 25, 3))

# Flags
#TODO - GRG : Need to ensure that the running variable is thread safe i.e. same value across multiple threads
running = False                          # Flag to indicate that triangulation algorithm is running
calib_done = False                       # Flag to indicate that the calibration procedure is done
transform_computed = False               # Flag to indicate that the transformation (R, t) was computed

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
Q3.2: Triangulate a set of 2D coordinates in the image to a set of 3D points.
    Input:  C1, the 3x4 camera matrix
            pts1, the Nx2 matrix with the 2D image coordinates per row
            C2, the 3x4 camera matrix
            pts2, the Nx2 matrix with the 2D image coordinates per row
    Output: P, the Nx3 matrix with the corresponding 3D points per row
            err, the reprojection error.
'''
def triangulate(C1, C2, pts1, pts2):
    # Replace pass by your implementation
    P = []
    err = 0
   
    for [x1, y1], [x2, y2] in zip(pts1, pts2):
        A = np.concatenate([
                 [y1*C1[2,:] - C1[1,:]]
                ,[C1[0,:] - x1*C1[2,:]]
                ,[y2*C2[2,:] - C2[1,:]]  
                ,[C2[0,:] - x2*C2[2,:]] 
        ])

        u,s,v = np.linalg.svd(A)
        p = v[-1]
        #p = (p/p[-1])

        p1_e = np.matmul(C1,p)
        p2_e = np.matmul(C2,p)
        p1_e = (p1_e/p1_e[-1])[:-1]
        p2_e = (p2_e/p2_e[-1])[:-1]
        
        
        # P.append(p[:-1])
        P.append(p)
    
        err += (np.linalg.norm([x1, y1]-p1_e)**2 + np.linalg.norm([x2, y2]-p2_e)**2)
    print(err)   
    return np.array(P), err


'''
triangulateTwoBodies: obtain 3D reconstruction of body given body coordinates from two different 
                      cameras.
    Input:  camera_1, projection matrix of first camera
            camera_2, projection matrix of second camera
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

    #Note - on cv2.triangulatePoints function, the 3D point is found from the equation that 
    # x = CX where x is 2D image point, X is the 3D world point, C is camera projection matrix (3D to 2D conversion matrix)
    # and the fact that x cross-profuct with PX must be zero, so can write the equations from these into a linear form as AX = 0,
    # and each camera in the stereo pair gives 2 equations thus A is 4x4 
    # Sub-note - all points are in homogenious coordinate in the above equations
    
    # For each
    pts4D = cv2.triangulatePoints(camera_1, camera_2, pt1, pt2).T
    pts4D = triangulate(camera_1, camera_2, pts1, pts2)
    #pts4D = cv2.triangulatePoints(camera_2, camera_1, pt2, pt1).T

    # Convert from homogeneous coordinates to 3D
    pts3D = pts4D[:, :3]/np.repeat(pts4D[:, 3], 3).reshape(-1, 3)


    #     neck_hip = distance_points(pts3D[1], pts3D[8])

    #     optimal_dist = 1
    #     factor = optimal_dist / neck_hip
    #     pts3D = pts3D[:, :3] * factor


    # pts3D = pts3D[:, :3] / (neck_hip*1.2)

    # Normalize 4D
    pts4D = pts4D[:, :4]/np.repeat(pts4D[:, 3], 4).reshape(-1, 4)

    # Get error
    error, error_mat = repro_error(pts4D, camera_1, camera_2, pt1, pt2)
    return error, error_mat, pts3D


# https://github.com/IntelRealSense/librealsense/blob/master/doc/depth-from-stereo.md
def triangulateStereo(body1, body2):
    fx = 940         # lense focal length (need to check for logitech)
    baseline = 1000  # distance in mm between two cameras (1m)
    units = 2500     # scaling factor

    # Store body coordinates in 3D
    pts3D = np.zeros((25, 3))

    # These two lines should be replaced to use the intrinsic matrix instead
    pts3D[:,0] = body1[:,0] / 600
    pts3D[:,1] = body1[:,1] / 700

    # Get disparity from two images
    disparity = (body2[:,0]-body1[:,0])

    # Compute depth
    depth = (fx * baseline) / (units * disparity)
    pts3D[:, 2] = depth
    return pts3D


def get3DPointsStereo(camera1, camera2):

    global y_offset

    num_bodies_cam1 = get_num_body_camera(camera1)
    num_bodies_cam2 = get_num_body_camera(camera2)
    num_bodies = 0

    # Hold the 3D coordinates after triangulation
    saved3DCoordinates = np.zeros((25, 3, num_bodies_cam1))

    #TODO - GRG : Limitation - Currently, we need the stereo pair to detect the same number of persons, 
    # which may not always be the case since field of view is slightly different
    if (num_bodies_cam1 != num_bodies_cam2) or (num_bodies_cam1 == 0):
        return saved3DCoordinates, 0

    # Sort bodies left to right
    # Camera1
    body_cam1_dict = {}
    for body in range(num_bodies_cam1):
        body_cam1_dict[body] = dataPoints[camera1, :, :, body][1, 0]
    body_cam1_dict = sorted(body_cam1_dict.items(), key=lambda x: x[1]) #TODO - GRG : Need to make sure that the sorting doesn't mess up the data
    # Camera2
    body_cam2_dict = {}
    for body in range(num_bodies_cam2):
        body_cam2_dict[body] = dataPoints[camera2, :, :, body][1, 0]
    body_cam2_dict = sorted(body_cam2_dict.items(), key=lambda x: x[1])

    # Perform triangulation
    for body in range(num_bodies_cam1):
        first_points = dataPoints[camera1, :, :, body_cam1_dict[body][0]]
        second_points = dataPoints[camera2, :, :, body_cam2_dict[body][0]]

        # Using stereo
        if (USE_STEREO_3D == True):
            pts3D = triangulateStereo(first_points, second_points)
            new_error = 100
        else:
            new_error, error_mat, pts3D = triangulateTwoBodies(projs[camera1], projs[camera2], first_points, second_points)

        print("[{}][{}] Body{}: {}".format(camera1, camera2, body, new_error))
        if (new_error < ERR_THRESHOLD):
            # print("GOOD [{}][{}] Body{}: {}".format(camera1, camera2, body, new_error))
            # Adjust y to put person on the ground if reference stereo camera
            # if (camera1 == 0):
            #     pts3D[:,1] -= y_offset
            #if (camera1 == 0):
            #    pts3D[:,1] = np.dot(m_matrices, pts3D)
            
            print(f'Body-{body} 3D Pts: {pts3D}')

            saved3DCoordinates[:,:,body] = pts3D
            num_bodies += 1

    return saved3DCoordinates, num_bodies


'''
Function to calculate the gitter in the 3D points i.e. the error in 3D poitns of a person compared to previous frame data
'''
import copy
#global saved3DCoordinates_previousFrame = np.empty((25, 3, total_skels*2))

saved3DCoordinates_previousFrame = np.empty((25, 3))


def calculateGitter_of_body0(saved3DCoordinates):
    global saved3DCoordinates_previousFrame

    gitter = np.linalg.norm(saved3DCoordinates - saved3DCoordinates_previousFrame, ord = 2)
    print(f'Gitter for body-0 is : {gitter}')

    saved3DCoordinates_previousFrame = copy.deepcopy(saved3DCoordinates)


'''
This calculates the Transformation matrix R & T from dataset A to dataset B.
Ref-links :
# http://nghiaho.com/?page_id=671
# https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
# Input: expects 3xN matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector
'''
def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    num_rows, num_cols = A.shape

    if num_rows != 3:
        raise Exception("matrix A is not 3xN, it is {}x{}".format(num_rows, num_cols))

    [num_rows, num_cols] = B.shape
    if num_rows != 3:
        raise Exception("matrix B is not 3xN, it is {}x{}".format(num_rows, num_cols))

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # subtract mean
    Am = A - np.tile(centroid_A, (num_cols, 1)).T
    Bm = B - np.tile(centroid_B, (num_cols, 1)).T

    # dot is matrix multiplication for array
    H = Am @ np.transpose(Bm)

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T #TODO - GRG : This calculates a transform from A to B, but don't we need transform from B to A?

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...\n");
        Vt[2,:] *= -1 #TODO - GRG : In the referenced articale the V & U are recalculated using the R. Need to check this.
        R = Vt.T @ U.T

    t = -R@centroid_A + centroid_B

    return R, t

''' 
Performs scaling followed by rotation and then translation
'''
def transformBody(body, R, t, scale = 1):
    body = body.T
    body = body * scale
    transformed_body = (R @ body) + np.tile(t, (25, 1)).T
    return transformed_body.T


'''
Used to find the best transformation matrix R & T between the 2 stereo sets. 
For each body correspondace transformation matrix is calculated and RANSAC algorithm is 
used to calculate the no of inliers (i.e. Body pairs/correspondences that agree with the transformation matrix).
The transformation matrix having the highest no of inliers is chosen
'''
def ransacH(pts_affine, num_iter=5000, tol=0.25):
    max_inliers = 0
    pts1_affine = pts_affine[0]
    pts2_affine = pts_affine[1]
    #TODO - GRG Ciritcal: How are we assuring the one-to-one body correspondance between the 2 stereo sets? It may so happen that body-1 in stereo set-1 may be actually body-2 in stereo set-2 and not body-1

    bestR = 0
    bestt = 0
    besty_offset = 0
    bestscale = 0

    # Iterate through each pair of 3D bodies
    for i in range(pts1_affine.shape[0]):
        # Select first body
        skel_1 = pts1_affine[i]
        skel_1[skel_1 < -10] = 0
        skel_1[skel_1 > 10] = 0

        # Select second body
        skel_2 = pts2_affine[i]
        skel_2[skel_2 < -10] = 0
        skel_2[skel_2 > 10] = 0

        # Obtain y offset to put skel_1 on the ground
        mean_foot = np.mean(skel_1[[11, 14, 19, 20, 21, 22, 23, 24],1])
        skel_1[:,1] -= mean_foot
        y_offset = mean_foot

        # Normalize the two skeletons?
        neck_hip_1 = distance_points(skel_1[1], skel_1[8])
        neck_hip_2 = distance_points(skel_2[1], skel_2[8])
        scale = neck_hip_1 / neck_hip_2

        if (np.isnan(scale).any()):
            scale = 1   #TODO - GRG : Ensure this only sets scale = 1 for bodies/skel which have Nan entries 

        # Scale second skeleton to size of first before finding the transformation
        skel_2 = scale * skel_2

        # Obtain R, t to go from skel_2 to skel_1
        R, t = rigid_transform_3D(skel_2.T, skel_1.T)

        num_inliers = 0
        for j in range(pts1_affine.shape[0]):
            # Verify error for each 3D body pairs
            skel_1_est = transformBody(pts2_affine[j], R, t, scale)
            err = np.sqrt(np.sum((skel_1_est - pts1_affine[j]) **2)) / 25 #TODO - GRG : Why divide by 25? because of 25 pose points?
            if (err < 0.25): #TODO - GRG : How is the error threshold determined as 0.25?
                num_inliers += 1

        print("[{}] inliers: {}/{} ({})".format(i, num_inliers, max_inliers, pts1_affine.shape[0]))

        if (num_inliers > max_inliers):
            max_inliers = num_inliers
            bestR = R
            bestt = t
            besty_offset = y_offset
            bestscale = scale

    print(f'Max inliers found is {max_inliers}')

    #TODO - GRG : Technically RANSAC algorithm recalculates the parameters again using all the inliers 

    return bestR, bestt, besty_offset, bestscale

# Function to obtain 3D skeletons using multiple stereo cameras
def convertTo3DPointsStereo(ref_cam, number_cameras):
    global transform_computed
    global num_skels_before_transform
    global R
    global t
    global y_offset
    global scale
    global pts_affine

    # Obtain 3D skeletons from first stereo (cam0-cam1)
    set1_3DCoordinates, num_body_set1 = get3DPointsStereo(0, 1)
    body_list_set1 = list(range(num_body_set1))

    # # Obtain 3D skeletons from second stereo (cam2-cam3)
    # set2_3DCoordinates, num_body_set2 = get3DPointsStereo(2, 3)
    # body_list_set2 = list(range(num_body_set2))

    # Total number of 3D skeleton across both cameras
    # total_skels = num_body_set1 + num_body_set2
    total_skels = num_body_set1

    # Will store 3D skeletons after fusion between the two stereo pairs
    saved3DCoordinates = np.empty((25, 3, total_skels*2))
    body_valid = 0

    # if (transform_computed == False) and (num_body_set1 == 1) and (num_body_set2 == 1):
    #     pts_affine[0][num_skels_before_transform] = set1_3DCoordinates[:, :, 0] 
    #     pts_affine[1][num_skels_before_transform] = set2_3DCoordinates[:, :, 0]
    #     num_skels_before_transform += 1
    #     print("{} 3D skels collected for affine RANSAC".format(num_skels_before_transform))

    # # If R, t have not be found, compute them
    # if (transform_computed == False) and (num_skels_before_transform == MIN_BODY_AFFINE):
    #     # Obtain R, t, y_offset and scale using RANSAC
    #     R, t, y_offset, scale = ransacH(pts_affine)
    #     print("R", R)
    #     print("t", t)
    #     print("scale", scale)
    #     print("y_offset", y_offset)

    #     #TODO - GRG : How is this error value calculated?
    #     err = 0.25

    #     # Save it for later
    #     np.savez(AFFINE_PATH, R = R, t=t, scale = scale, y_offset = y_offset, err=err, date=int(time.time()))

    #     # Set transform_computed flag to True
    #     transform_computed = True

    # #TODO - GRG : Fusion needs to be performed once the transform is computed 
    # #TODO - GRG : but it is skipped for the transform computation step which needs to be taken care of
    
    # # Perform fusion of skeletons from different stereos
    # elif (transform_computed == True):
    
    # Send any skeletons from the first camera pair
    for body_set1 in body_list_set1:
    	# Select person from first stereo
    	# person_set1 = set1_3DCoordinates[:, :, body_set1]

    	# #TODO - GRG : Optimization - Can avoid this for loop entirely using matrix operation instead 
    	# # If body_set1 and body_set2 similar, draw best one and remove from list
    	# for body_set2 in body_list_set2:
    	#     # Transform person from second stereo to match first stereo coordinates
    	#     person_set2 = set2_3DCoordinates[:, :, body_set2]
    	#     person_set2_transformed = transformBody(person_set2, R, t, scale)

    	#     # Compute difference of neck position and remove from second list if less than 3
    	#     diff = np.abs(np.sum(person_set1[1]-person_set2_transformed[1]))
    	#     print("diff: ", diff)
    	#     if (diff < 1): #TODO - GRG : How is this value determined?
    	#         print("________________DIFF < 1: REMOVE BODY 2", diff)
    	#         body_list_set2.remove(body_set2) #TODO - GRG : Why are we preferring stereo set-1 over set-2?
    	# Send skeleon from set1
    	print("[1] Send body{}".format(body_set1))
    	saved3DCoordinates[:,:,body_valid] = set1_3DCoordinates[:, :, body_set1] #TODO - GRG : Why are we considering/sending all of the body corresponding to set-1?
    	body_valid += 1

    	# # Any remaining skeleton from second camera pair
    	# for body_set2 in body_list_set2:
    	#     # print("[2] Send body{}".format(body_set2))
    	#     # Select body
    	#     person_set2 = set2_3DCoordinates[:, :, body_set2]

    	#     # Transform body
    	#     person_set2_transformed = transformBody(person_set2, R, t, scale)

    	#     # Send it
    	#     saved3DCoordinates[:,:,body_valid] = person_set2_transformed
    	#     body_valid += 1
    	#     # saved3DCoordinates[:,:,body_valid] = A
    	#     # body_valid += 1

    # Fill data Points with 0
    dataPoints.fill(0) #Note - GRG : Global varible holding the deteched data points which needs to be cleared once processed

    # In case the number of people is greater than MAX_NUM_PEOPLE supported (wrong fusion), don't send anything
    if (body_valid >= MAX_NUM_PEOPLE):
        print(f'Error!!! Wrong fusion occurred as {body_valid} detected bodies found which is greater than {MAX_NUM_PEOPLE} Max No of detection allowed...')
        body_valid = 0

    # print("Number of bodies to be sent:", body_valid)

    #Print the gitter 
    if saved3DCoordinates.shape[-1] > 0:
        calculateGitter_of_body0(saved3DCoordinates[:,:,0])

    return saved3DCoordinates, body_valid

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

    # Dictionary containing the list of bodies in each camera
    bodies_id_cam = {}
    for cam in list_cams:
        bodies_id_cam[cam] = list(range(get_num_body_camera(cam)))

    print(bodies_id_cam)

    list_cams.remove(ref_cam)


    list_cam_chosen = []
    
    # Get number of people: assume visibile in three cameras
    num_bodies_ref_cam = get_num_body_camera(ref_cam)
    
    # Hold the 3D coordinates after triangulation
    saved3DCoordinates = np.empty((25, 3, num_bodies_ref_cam))

    # TODO: Order people by looking at the neck x coordinate and sort them from left to right


    not_equal = False

    if (bodies_id_cam[0] != bodies_id_cam[1]):
        not_equal = True
    
    # Go through each body in camera 0
    for body_num_cam_ref in bodies_id_cam[ref_cam]:
        error = 10000000000
        send = False
        first_points = dataPoints[ref_cam, :, :, body_num_cam_ref]
        for camera_num in list_cams:
            for body_num in bodies_id_cam[camera_num]:
                # Triangulate points
                second_points = dataPoints[camera_num, :, :, body_num]
                new_error, error_mat, pts3D = triangulateTwoBodies(projs[ref_cam], projs[camera_num], first_points, second_points)
                # print(camera_num, body_num_cam_ref, body_num, new_error)
                # print(first_points[3], second_points[3])
                print("BNUM_REF {}, CAM{}, BNUM {}, {}".format(body_num_cam_ref, camera_num, body_num, new_error))
                
                # If triangulation error is less than the previous and the max threshold, send it
                if ((new_error < error) and (new_error < ERR_THRESHOLD)):
                    send = True
                    saved3DCoordinates[:,:,num_body_ok] = pts3D
                    cam_chosen = camera_num
                    body_num_chosen = body_num
                    error = new_error


        if (send):
            print("[{}]: cam{}, body{}, error:{}".format(body_num_cam_ref, cam_chosen, body_num_chosen, error))
            list_cam_chosen.append(cam_chosen)
            num_body_ok += 1
            # Fix same person
            bodies_id_cam[cam_chosen].remove(body_num_chosen) 

    num_bodies_cam1 = get_num_body_camera(0)
    num_bodies_cam2 = get_num_body_camera(1)
    if (num_bodies_cam1 == num_bodies_cam2):
        saved3DCoordinates, num_body_ok = get3DPointsStereo(0, 1)
    

    # Fill data Points with 0
    dataPoints.fill(0)

    if (not_equal):
            saved3DCoordinates = np.empty((25, 3, num_bodies_ref_cam))
            num_body_ok = 0
            list_cam_chosen = []

    print("_____________________")

    return saved3DCoordinates, num_body_ok, list_cam_chosen

'''
get_num_body_camera: helper function to find number of bodies detected by a camera
    Input:  camera_num, camera of interest
    Output: body_count, number of bodies found by that camera in that frame
'''
def get_num_body_camera(camera_num):
    body_count = 0
    for i in range(dataPoints.shape[3]):
        if (np.any(dataPoints[camera_num,:,:,i]) == True): #Note - GRG : Any non-zero value or non-false value will satisfy the if condition
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
#TODO - GRG : Check this method
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
                # print(lambda_id, ts, person_id)
                if (score > CONF_SCORE):
                    if (running):
                        print("----------------------ALERT RUNNING")
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
    # zero_coord = np.array([0, 0, 0, 1])      
    zero_coord = np.array([0, 0, 0])      
    # point = np.dot(projection, zero_coord)
    point = np.dot(projection[:,:-1].T, zero_coord-projection[:,-1])
    message = json.dumps(point.flatten().tolist())
    message = message.strip('[]')
    return message

######################### END #########################

    
if __name__ == '__main__':
    pts_calib = {new_list: [] for new_list in range(NUM_CAMERAS)} 
    time_calib = {new_list: [] for new_list in range(NUM_CAMERAS)} 

    if (RUN_CALIBRATION == False):
        calib_done = True
        transform_computed = True

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

    # Create empty numpy array to hold projection matrices
    projs = np.empty((NUM_CAMERAS, 3, 4))
    m_matrices = np.empty((NUM_CAMERAS, 3, 4))

    # Default intrinsic matrices
    K1 = K2 = np.load(INTRINSICS_PATH)["mtx"]     

    # Load projection matrices
    if (RUN_CALIBRATION == False):
        prev_projs = np.load(PROJS_PATH)
        dt_object = datetime.fromtimestamp(prev_projs["date"])
        date_time = dt_object.strftime("%m/%d/%Y, %H:%M:%S")
        projs = prev_projs["projs"]
        m_matrices = prev_projs["m_matrices"]
        print("Loaded projections for {} cameras from [{}]".format(prev_projs["num_cameras"], date_time))
        print(f'Loaded projections of camera-1 : {m_matrices[0]}')
        print(f'Loaded projections of camera-2 : {m_matrices[1]}')
    # Compute new projection matrices
    else:
        print("Running autocalibration")
        # Iterate through each pair of camera
        # for camera_num in range(NUM_CAMERAS-1, 2):
        for camera_num in [0, 2]:
            print("Calib cam{}-cam{}".format(camera_num, camera_num+1))
            print("Camera{}: {} bodies collected".format(camera_num, len(pts_calib[camera_num])))
            print("Camera{}: {} bodies collected".format(camera_num+1, len(pts_calib[camera_num+1])))
            projs[camera_num], projs[camera_num + 1], m_matrices[camera_num], m_matrices[camera_num + 1] = calibration.get_projs_matrices(pts_calib[camera_num], pts_calib[camera_num + 1], K1, K2)

        # Save resulting projection matrices
        np.savez(PROJS_PATH, num_cameras = NUM_CAMERAS, K1=K1, K2=K2, projs=projs, m_matrices = m_matrices,date=int(time.time()))

    baseline = 1
    camera_ht = 1.8
    #TODO - GRG : Manually setting the camera matrix of stereo pair 1 for now

    # m_matrices[0] = np.array(
    #                         [[-1,0,0,0],
    #                          [0,-1,0,y_offset],
    #                          [0,0,1,0]],
    #                          dtype='float64'
    #                         )

    # m_matrices[1] = np.array(
    #                         [[-1,0,0,baseline],
    #                          [0,-1,0,y_offset],
    #                          [0,0,1,0]],
    #                          dtype='float64'
    #                         )

    R1 = R2 = np.array(
                        [[-1,0,0],
                         [0,-1,0],
                         [0,0,1]],
                         dtype='float64'
                        )
    T1 = np.array([0, camera_ht, 0]).T
    T2 = np.array([baseline, camera_ht, 0]).T
    
    # m_matrices[0] = np.column_stack((R1, np.dot(R1, T1)))
    # m_matrices[1] = np.column_stack((R2, np.dot(R2, T2)))
    m_matrices[0] = np.column_stack((R1, T1))
    m_matrices[1] = np.column_stack((R2, T2))

    #Note : Projection matrix is transformation from 3D coordinate to 2D coordinate system
    projs[0] = np.dot(K1, m_matrices[0])
    projs[1] = np.dot(K2, m_matrices[1])

    if (transform_computed):
        affine = np.load(AFFINE_PATH)
        R = affine["R"]
        t = affine["t"]
        scale = affine["scale"]
        y_offset = affine["y_offset"]
        print("Error from loaded affine transform: ", affine["err"])
        print("R", R)
        print("t", t)
        print("scale", scale)
        print("y_offset", y_offset)

    ################## INITIALIZATION BEFORE ALGORITHM ################## 
    # Hold people coordinates
    prev3DPoints = np.empty((25, 3, MAX_NUM_PEOPLE))

    # Hold current frame number for averaging
    frame = 0

    # Initialize mapping function
    id_mapping = {}
    for i in range(MAX_NUM_PEOPLE):
        id_mapping[i] = i

    # MQTT loop
    while True:
        # Condition to start triangulation
        start_triangulate = all(received_data) # Data have been received from each cameras

        # Triangulate
        if (start_triangulate):
            # Reset array that stores whether data were received for a particular lambda
            received_data = [False] * NUM_CAMERAS

            if (not running):
                # Make sure we don't run that algorithm conccurently
                running = True

                # Publish camera coordinates to be displayed in ARENA
                for i in range(NUM_CAMERAS):
                    client.publish(TOPIC_CAMERA + str(i), publish_camera(m_matrices[i])) 

                # 1. Obtain 3D coordinates of people using triangulation
                points3D, numBodies = convertTo3DPointsStereo(REF_CAM, NUM_CAMERAS)

                # 2. Filter new body that arrived
                # for body_num in range(numBodies):
                    # filter_body(points3D[:,:,body_num])

                # 3. Find new mapping of people id
                id_mapping, prev = find_new_mapping(prev3DPoints, points3D, numBodies, id_mapping)
                
                # Publish each body separately
                for person in range(numBodies):                
                    # Fusion prev body and new body
                    # !!! Might need to fusion in find_new_mapping and take into account id_mapping
                    # prev3DPoints[:,:,person] = fusion_body(prev3DPoints[:,:,person], points3D[:,:,person])
                    # prev3DPoints[:,:,person] = points3D[:,:,person]

                    #TODO - GRG : May need to place this if-condition before the for-loop to optimize performance
                    # Done averaging, publish it
                    if (frame == FRAME_AVERAGING):
                        # Filter body (after fusion?)
                        filter_body(points3D[:,:,id_mapping[person]])

                        # Count number of zero elements in the points3D
                        nz = np.count_nonzero(points3D[:,:,id_mapping[person]])
                        if (nz > MIN_BODY_POINTS):
                            print("[{}] Person{}[{}]: NZ parts = {} ------------- SENDING".format(time.time(), person, id_mapping[person], nz))
                            client.publish(TOPIC_3D, publish_person(points3D[:,:,id_mapping[person]], person)) 
                        else:
                            print("Person{}[{}]: NZ parts = {}".format(person, id_mapping[person], nz))


                if (frame == FRAME_AVERAGING):
                    frame = 0
                else:
                    frame +=1

                # Replace previous points with new points and 0 for the rest
                for person in range(numBodies): #Update pose points for detected people
                    prev3DPoints[:,:,person] = points3D[:,:,person]
                for person in range(numBodies, MAX_NUM_PEOPLE): #For people who haven't been detected now clear them out be filling 0
                    prev3DPoints[:,:,person].fill(0)

                # Done with this funtion
                running = False
        # End received frames from lambdas

        # Get new messages
        client.loop(0.01) #TODO - GRG Note : Max Frame rate at which posefusion is running i.e. 10ms or 100FPS