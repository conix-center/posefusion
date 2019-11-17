import time
import random
import numpy 
import paho.mqtt.client as paho
broker="oz.andrew.cmu.edu"
object_path="/topic/skeleton"
draw_path="/topic/openpose"
TOPIC_CAMERA    = "/posefusion/camera/+"

FORMAT="{0:0.3f}"
lastx=0.0
lasty=0.0
lastz=0.0
x=0.0
y=0.0
z=0.0
MESSAGE=""

global prev
prev = 0

# 0: red, 1: green, 2: blue
color_person = {
    "Person0": "#FF0000", 
    "Person1": "#00FF00",
    "Person2": "#0000FF",
}

def tryDraw(person,bodypart,arr, color):
    counter = 0
    onoff = "on"

    # Check if the body part should be drawn or not
    while (counter < len(arr)):
        x = -arr[counter][0]
        y = arr[counter][1]
        z = arr[counter][2]
        if (x == 0) and (y == 0) and (z == 0):
            print("{} --- {} OFF ".format(person, bodypart))
            onoff = "off"
        counter = counter + 1

    counter = 0
    while (counter < len(arr)):
        x = -arr[counter][0]
        y = 1-arr[counter][1]
        z = arr[counter][2]

        if (counter == 0):
            lastx=x
            lasty=y
            lastz=z
            
        partID = person+'_'+bodypart+'_'+str(counter)
        MESSAGE="thickline_"+partID+","+FORMAT.format(lastx)+','+FORMAT.format(lasty)+','+FORMAT.format(lastz)+','+FORMAT.format(x)+','+FORMAT.format(y)+','+FORMAT.format(z)+",0,12,1,1,"+color+","+onoff
        client.publish(draw_path+"/thickline_"+partID,MESSAGE)
        # print("{} x:{} y:{} z:{}".format(partID, x, y, z))
        # print(         draw_path+"/thickline_"+partID,MESSAGE)
        # print(" ")
        lastx=x
        lasty=y
        lastz=z
        counter = counter + 1

def on_connect(client, userdata, flags, rc):
    print("connected with result code "+str(rc))
    client.subscribe(object_path) #subscribe
    client.subscribe(TOPIC_CAMERA) #subscribe

#define callback: listen for /topic/skeleton/person0 messages
# send out draw commands on draw_path that match their coordinates
def on_message(client, userdata, message):

    if (message.topic[:-1] == TOPIC_CAMERA[:-1]):
        theMessage=str(message.payload.decode("utf-8"))
        splits=theMessage.split(',')
        x = float(splits[0])
        y = float(splits[1])
        z = float(splits[2])

        camera = int(message.topic[-1])
        if (camera == 0):
            color = "#FF0000"
        elif (camera == 1):
            color = "#00FF00"
        else:
            color = "#0000FF"

        # print(splits)
        # cam_coord="thickline_"+","+FORMAT.format(float(splits[0]))
        # person = "Person10"
        # bodypart = str(int(message.topic[-1]))
        # counter = 0
        # partID = person+'_'+bodypart+'_'+str(counter)
        # cam_coord = "thickline_"+partID+","+FORMAT.format(splits[0])+','+FORMAT.format(splits[1])+','+FORMAT.format(splits[2])+','+FORMAT.format(splits[0]+0.1)+','+FORMAT.format(splits[1]+0.1)+','+FORMAT.format(splits[2]+0.1)+",0,12,1,1,"+"#00FF00"+","+"on"
        # print(cam_coord)
        cam_coord = "cube_cam"+ str(camera) +","+FORMAT.format(x)+","+FORMAT.format(y)+","+FORMAT.format(z)+","+"0"+","+"0"+","+"0"+","+"0"+","+"0.2"+","+"0.2"+","+"0.2"+","+color+","+"on"
        client.publish(draw_path+"/cube_cam" + str(camera),cam_coord)
        # client.publish(draw_path+"/thickline_"+partID,cam_coord)
    else:
        # before = time.time()
        theMessage=str(message.payload.decode("utf-8"))
        splits=theMessage.split(',')

        # splits[0] is the name e.g. "Person_1"
        #   client.publish(draw_path,"thickline_1,2,2,2,3,3,3,0,0,0,0,#CE00FF,on")
        Person = splits[0]

        # if (Person == "Person0") or (Person == "Person1"):
        if (True):
            # print(Person + " " + str((time.time()-prev)*1000) + "ms")
            print(Person + " " + str(time.time()))

            # if (Person == "Person0"):
            Nose = [float(splits[1]), float(splits[2]), float(splits[3])]
            Neck = [float(splits[4]), float(splits[5]), float(splits[6])]
            RShoulder = [float(splits[7]), float(splits[8]), float(splits[9])]
            RElbow = [float(splits[10]), float(splits[11]), float(splits[12])]
            RWrist = [float(splits[13]), float(splits[14]), float(splits[15])]
            LShoulder = [float(splits[16]), float(splits[17]), float(splits[18])]
            LElbow = [float(splits[19]), float(splits[20]), float(splits[21])]
            LWrist = [float(splits[22]), float(splits[23]), float(splits[24])]
            MidHip = [float(splits[25]), float(splits[26]), float(splits[27])]
            RHip = [float(splits[28]), float(splits[29]), float(splits[30])]
            RKnee = [float(splits[31]), float(splits[32]), float(splits[33])]
            RAnkle = [float(splits[34]), float(splits[35]), float(splits[36])]
            LHip = [float(splits[37]), float(splits[38]), float(splits[39])]
            LKnee = [float(splits[40]), float(splits[41]), float(splits[42])]
            LAnkle = [float(splits[43]), float(splits[44]), float(splits[45])]
            REye = [float(splits[46]), float(splits[47]), float(splits[48])]
            LEye = [float(splits[49]), float(splits[50]), float(splits[51])]
            REar = [float(splits[52]), float(splits[53]), float(splits[54])]
            LEar = [float(splits[55]), float(splits[56]), float(splits[57])]
            LBigToe = [float(splits[58]), float(splits[59]), float(splits[60])]
            LSmallToe = [float(splits[61]), float(splits[62]), float(splits[63])]
            LHeel = [float(splits[64]), float(splits[65]), float(splits[66])]
            RBigToe = [float(splits[67]), float(splits[68]), float(splits[69])]
            RSmallToe = [float(splits[70]), float(splits[71]), float(splits[72])]
            RHeel = [float(splits[73]), float(splits[74]), float(splits[75])]

            # tryDraw( Person, "leftHead", [ Nose, LEye, LEar ], color_person[Person])
            # tryDraw( Person, "rightHead", [ Nose, REye, REar ], color_person[Person])
            # tryDraw( Person, "torso", [ Nose, Neck, MidHip ], color_person[Person])
            # tryDraw( Person, "leftArm", [ Neck, LShoulder, LElbow, LWrist ], color_person[Person])
            # tryDraw( Person, "rightArm", [ Neck, RShoulder, RElbow, RWrist ], color_person[Person])
            # tryDraw( Person, "leftLeg", [ MidHip, LHip, LKnee, LAnkle ], color_person[Person])
            # tryDraw( Person, "rightLeg", [ MidHip, RHip, RKnee, RAnkle ], color_person[Person])
            # tryDraw( Person, "leftFoot", [ LHeel, LAnkle, LBigToe, LSmallToe ], color_person[Person])
            # tryDraw( Person, "rightFoot", [ RHeel, RAnkle, RBigToe, RSmallToe ], color_person[Person])

            tryDraw( Person, "leftHead", [ Nose, LEye, LEar ], "#280B68" )
            tryDraw( Person, "rightHead", [ Nose, REye, REar ], "#740C66" )
            tryDraw( Person, "torso", [ Nose, Neck, MidHip ], "#790A07" )
            tryDraw( Person, "leftArm", [ Neck, LShoulder, LElbow, LWrist ], "#429A1D" )
            tryDraw( Person, "rightArm", [ Neck, RShoulder, RElbow, RWrist ], "#818525" )
            tryDraw( Person, "leftLeg", [ MidHip, LHip, LKnee, LAnkle ], "#113887" )
            tryDraw( Person, "rightLeg", [ MidHip, RHip, RKnee, RAnkle ], "#1B9E92" )
            tryDraw( Person, "leftFoot", [ LHeel, LAnkle, LBigToe, LSmallToe ], "#FFFFFF" ) #"#0A0769"
            tryDraw( Person, "rightFoot", [ RHeel, RAnkle, RBigToe, RSmallToe ], "#FFFFFF" ) #"#0A0769"

            # print("COCO : " + Person + ""+ str((time.time()-before)*1000) + "ms")
            prev = time.time()

client= paho.Client()

######Bind function to callback
client.on_connect=on_connect
client.on_message=on_message
#####
print("connecting to broker ",broker)
client.connect(broker)
#connect
client.loop_start() #start loop to process received messages

while True:
    # do nothing
    time.sleep(0.1)

client.disconnect() #disconnect
client.loop_stop() #stop loop
