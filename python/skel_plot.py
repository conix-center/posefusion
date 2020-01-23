from matplotlib import pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation

import time
import random
import paho.mqtt.client as paho
import json

SERVER_ADDR     = "oz.andrew.cmu.edu"
TOPIC_3D        = "/posefusion/skeleton"
TOPIC_CAMERA    = "/posefusion/camera/+"
draw_path = "/topic/openpose"
scene = "openpose"

FORMAT="{0:0.3f}"
MESSAGE=""


data = np.array([[2.09103881, 1.27165952, 3.15332506],
 [2.18702306, 1.13109862, 3.25948327],
 [2.30931267, 1.14221703, 3.20557387],
 [2.48419121, 0.86507568, 3.31679331],
 [2.32625412, 0.72966878, 3.2062737 ],
 [2.04428246, 1.1195167,  3.3167503 ],
 [2.00953879, 0.92367406, 3.49583178],
 [2.06093462, 0.78834597, 3.34316737],
 [2.21505674, 0.68533923, 3.31385834],
 [2.3192124,  0.68048281, 3.31484991],
 [2.26319131, 0.52960803, 3.09497744],
 [2.51967513, 0.1276179,  3.49810469],
 [2.12196072, 0.68820973, 3.34363122],
 [1.95521795, 0.578667,   3.40772792],
 [1.91127413, 0.305938,   3.27654601],
 [2.10640951, 1.29673972, 3.12725301],
 [2.06651414, 1.2842772,  3.15382285],
 [2.16527354, 1.29244242, 3.15376995],
 [2.04255038, 1.25879157, 3.20563477],
 [1.93262955, 0.22804152, 3.28798198],
 [0.8492351,  1.86298544, 0.58336798],
 [1.90496106, 0.32175345, 3.22358046],
 [2.45251525, 0.17222233, 3.40488677],
 [2.47008478, 0.17227538, 3.40448639],
 [2.5477294, 0.09686418, 3.56256195]])


# data = np.array([[-0.36523693,  1.79980623, -4.00613491], [-0.34458465,  2.02875364, -4.00618081], [-0.49584682,  2.02688218, -3.88472047], [-0.55483389,  2.31398274, -3.87908215], [-0.61770946,  2.60244432, -3.83558038], [-0.17298607,  2.0303896 , -4.13470764], [-0.11051006,  2.3370799 , -4.22111817], [-0.04221266,  2.59813864, -4.04204286], [-0.31897148,  2.63268702, -3.95743895], [-0.45245289,  2.63261581, -3.95711129], [-0.49671577,  3.10810661, -4.07518918], [-0.60422296,  3.68573902, -4.39723786], [-0.20815007,  2.64150264, -3.99885908], [-0.19874207,  3.09033184, -4.0372718 ], [-0.13554613,  3.48907193, -4.0006965 ], [-0.38616241,  1.77881081, -4.00543734], [-0.31650741,  1.7784174 , -4.04650313], [-0.4436982 ,  1.79447512, -3.96412863], [-0.2908168 ,  1.7888427 , -4.13380848], [-0.13448431,  3.57559662, -3.96211404], [-0.06048883,  3.50194461, -3.84216065], [-0.15932684,  3.54587495, -4.08454844], [-0.62990157,  3.83515252, -4.4760605 ], [-0.71745346,  3.8438386 , -4.52138302], [-0.6063095 ,  3.7642258 , -4.49894635]])

def gen(n):
    phi = 0
    while phi < 2*np.pi:
        yield np.array([np.cos(phi), np.sin(phi), phi])
        phi += 2*np.pi/n

def update(num, ax, data1):
    global data
    ax.clear()
    drawSkel(ax)


def plotPoint(ax, data):
    ax.scatter3D(data[:,0], data[:,1], data[:,2], 'blue')


def plotline(ax, data, color):
    data = np.array(data)
    ax.plot3D(data[:,0], data[:,1], data[:,2], 'grey')


def drawSkel(ax):

    global data 
    Nose = data[0,:]
    Neck = data[1,:]
    RShoulder = data[2,:]
    RElbow = data[3,:]
    RWrist = data[4,:]
    LShoulder = data[5,:]
    LElbow = data[6,:]
    LWrist = data[7,:]
    MidHip = data[8,:]
    RHip = data[9,:]
    RKnee = data[10,:]
    RAnkle = data[11,:]
    LHip = data[12,:]
    LKnee = data[13,:]
    LAnkle = data[14,:]
    REye = data[15,:]
    LEye = data[16,:]
    REar = data[17,:]
    LEar = data[18,:]
    LBigToe = data[19,:]
    LSmallToe = data[20,:]
    LHeel = data[21,:]
    RBigToe = data[22,:]
    RSmallToe = data[23,:]
    RHeel = data[24,:]



    # refactorDraw( Person, "leftHead", [ Nose, LEye, LEar ], "#280B68" )
    # refactorDraw( Person, "rightHead", [ Nose, REye, REar ], "#740C66" )
    # refactorDraw( Person, "torso", [ Nose, Neck, MidHip ], "#790A07" )
    # refactorDraw( Person, "leftArm", [ Neck, LShoulder, LElbow, LWrist ], "#429A1D" )
    # refactorDraw( Person, "rightArm", [ Neck, RShoulder, RElbow, RWrist ], "#818525" )
    # refactorDraw( Person, "leftLeg", [ MidHip, LHip, LKnee, LAnkle ], "#113887" )
    # refactorDraw( Person, "rightLeg", [ MidHip, RHip, RKnee, RAnkle ], "#1B9E92" )
    # refactorDraw( Person, "leftFoot", [ LHeel, LAnkle, LBigToe, LSmallToe ], "#FFFFFF" ) #"#0A0769"
    # refactorDraw( Person, "rightFoot", [ RHeel, RAnkle, RBigToe, RSmallToe ], "#FFFFFF" ) #"#0A0769"

    plotPoint(ax, data)

    plotline(ax, [ Nose, LEye, LEar ], "#280B68")
    plotline(ax, [ Nose, REye, REar ], "#740C66" )
    plotline(ax, [ Nose, Neck, MidHip ], "#790A07" )
    plotline(ax, [ Neck, LShoulder, LElbow, LWrist ], "#429A1D" )
    plotline(ax, [ Neck, RShoulder, RElbow, RWrist ], "#818525" )
    plotline(ax, [ MidHip, LHip, LKnee, LAnkle ], "#113887" )
    plotline(ax, [ MidHip, RHip, RKnee, RAnkle ], "#1B9E92" )
    plotline(ax, [ LHeel, LAnkle, LBigToe, LSmallToe ], "#FFFFFF" ) #"#0A0769"
    plotline(ax, [ RHeel, RAnkle, RBigToe, RSmallToe ], "#FFFFFF" ) #"#0A0769"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(TOPIC_3D) #subscribe
    # client.subscribe(TOPIC_CAMERA) #subscribe

#define callback: listen for /topic/skeleton/person0 messages
# send out draw commands on draw_path that match their coordinates
def on_message(client, userdata, message):
    global data
    
    theMessage=str(message.payload.decode("utf-8"))
    splits=theMessage.split(',')

    Person = splits[0]
    print(Person + " " + str(time.time()))
    if(Person == 'Person0'):
        data = np.array(splits[1:-1], dtype = 'float64').reshape(-1,3) 
    

       




if __name__ == '__main__':

    client = paho.Client()
    client.on_connect=on_connect
    client.on_message=on_message

    ################## MQTT Connect ##################
    print("Connecting to broker: ", SERVER_ADDR)
    client.connect(SERVER_ADDR)
    client.loop_start() # start loop to process received messages


    fig = plt.figure()
    ax = p3.Axes3D(fig)
    
    N = 100

    # Setting the axes properties
    #ax.set_xlim3d([-1.0, 1.0])
    ax.set_xlabel('X')

    #ax.set_ylim3d([-1.0, 1.0])
    ax.set_ylabel('Y')

    #ax.set_zlim3d([0.0, 10.0])
    ax.set_zlabel('Z')

    ani = animation.FuncAnimation(fig, update, N, fargs=(ax, data), interval=10000/N, blit=False)
    #ani.save('matplot003.gif', writer='imagemagick')
    plt.show()


    client.disconnect() #disconnect
    client.loop_stop() #stop loop
    print('Program Stopped')


