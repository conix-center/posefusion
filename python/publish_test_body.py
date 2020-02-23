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


######################### MQTT Functions #########################
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))


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


######################### END #########################

    
if __name__ == '__main__':

    ################## MQTT Init ################## 
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    ################## MQTT Connect ################## 
    print("Connecting to broker: ", SERVER_ADDR)
    client.connect(SERVER_ADDR, port=1883, keepalive=60)
    print("Connected to broker: ", SERVER_ADDR)
    

    ################## INITIALIZATION BEFORE ALGORITHM ################## 
    # messageA = 'Person1,-0.4546721012585256, 1.6704650597972661, 3.1284286502907133, -0.4507804733630362, 1.5309047272280274, 3.3438826680505516, -0.2860607573830554, 1.5306556152506237, 3.3448499109043355, -0.20740651955008393, 1.235695525060224, 3.3438360274212706, -0.17168331757297442, 0.9900637810585478, 3.3166617980340667, -0.5973103111213394, 1.5396575310399125, 3.287872931028755, -0.678146921882364, 1.2468967850876986, 3.435288690989055, -0.7489320978859699, 1.023809308462359, 3.43468843532245, -0.4852208057702188, 0.9914183598276458, 3.402077717023118, -0.3826501103693077, 0.9706857872000875, 3.4330581789192403, -0.3858383495798725, 0.6164966303603494, 3.4645202029651267, 0.0, 0.0, 0.0, -0.6051233473842802, 0.9891721342403125, 3.4311888254994263, -0.6419827690791929, 0.6317916725211074, 3.3433075711371254, -0.6409244833010941, 0.26742025798311775, 3.2057683812655458, -0.4284171278433915, 1.7014801033760645, 3.177467456661561, 0.0, 0.0, 0.0, -0.36635295964992187, 1.7338167434677731, 3.2066490134362775, 0.0, 0.0, 0.0, -0.6239235370711037, 0.2673701272895639, 3.205596924719015, -0.6966061642848869, 0.2544999182887391, 3.232438914931758, -0.627674392805405, 0.27955826796112565, 3.180147702960898, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,on'
    messageA = 'Person8,-0.4546721, 1.67046505, 3.12842865, -0.450780473, 1.53090472, 3.34388266, -0.286060, 1.530655, 3.3448499, -0.2074065, 1.23569552, 3.3438360, -0.1716833, 0.9900637, 3.316661798, -0.5973103, 1.5396575, 3.287872, -0.6781469, 1.2468967, 3.4352886, -0.74893209, 1.023809, 3.43468, -0.485220, 0.991418, 3.402077, -0.382650, 0.9706857, 3.43305817, -0.38583834, 0.616496, 3.464520, 0.0, 0.0, 0.0, -0.605123, 0.98917213, 3.431188, -0.6419827, 0.631791, 3.343307, -0.64092, 0.267420, 3.2057683, -0.428417127, 1.70148, 3.17746, 0.0, 0.0, 0.0, -0.366352959, 1.733816, 3.206649, 0.0, 0.0, 0.0, -0.623923, 0.267370, 3.2055969, -0.696606164, 0.254499, 3.23243891, -0.627674, 0.2795582, 3.1801477, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,on'

    messageB = 'Person9,0.7463153715133708, 1.7347424733839822, 3.3722894475753313, 0.676520115892049, 1.5395261458903413, 3.4652552228285978, 0.7953095978618075, 1.5388792971341467, 3.5967151743292343, 0.8183924105291064, 1.2622434512123535, 3.6980041108606483, 0.8939128400296005, 1.0196776320333965, 3.7321348096030773, 0.5534731171942452, 1.5416901330465236, 3.374571292317819, 0.501047114019704, 1.2480088264663374, 3.3747760693727016, 0.5265164178895646, 0.9899448421213327, 3.3170653472472518, 0.7140683126095478, 1.0000009860665944, 3.5607875616919893, 0.8008632549954198, 0.9822482051876013, 3.6614673712205197, 0.818201268568752, 0.5745759084529024, 3.699101805688811, 0.49606746385728634, 0.3164120613634569, 3.8074484035606333, 0.6348327041350048, 1.0237386382985243, 3.434708268461999, 0.7747001645595925, 0.6577348571938627, 3.3441395238681095, 0.780438811582558, 0.35125003843968466, 3.260933866439034, 0.0, 0.0, 0.0, 0.7042930314104325, 1.762359052385663, 3.261083314438253, 0.0, 0.0, 0.0, 0.6294204956678053, 1.7562355329561954, 3.4046419124025147, 0.7817563130055247, 0.3998004795674145, 3.002587897515623, 0.7351160381144789, 0.39566083237920185, 3.0026983175697577, 0.761564646528856, 0.32217890032174024, 3.2877872046787346, 0.5244419903531574, 0.22759713873492068, 3.592040966471622, 0.5597450412440196, 0.16941339425074572, 3.769484578085319, 0.43213424047576604, 0.3347520826483643, 3.77333510558087,on'
    
    switch = False
    # MQTT loop
    while True:

        if switch:
            client.publish(TOPIC_3D, messageA, 0)
            # time.sleep(2)
        else:
            client.publish(TOPIC_3D, messageB, 0)
            # time.sleep(2)
        
        switch = not switch

        # Get new messages
        client.loop(1) #TODO - GRG Note : Max Frame rate at which posefusion is running i.e. 10ms or 100FPS
        time.sleep(5)

                