import time
import random
import numpy 
import paho.mqtt.client as paho
import json

SERVER_ADDR     = "oz.andrew.cmu.edu"
TOPIC_3D        = "/posefusion/skeleton"
TOPIC_CAMERA    = "/posefusion/camera/+"
draw_path = "/topic/openpose"
scene = "openpose"

last_ts_person = {}
for i in range(5):
    last_ts_person["Person" +str(i)] = 0

last_ts_check  = 0

FORMAT="{0:0.3f}"
lastx=0.0
lasty=0.0
lastz=0.0
x=0.0
y=0.0
z=0.0
MESSAGE=""

# 0: red, 1: green, 2: blue
color_person = {
    "Person0": "#FF0000", 
    "Person1": "#00FF00",
    "Person2": "#0000FF",
}

def refreshArena():
    t_now = time.time()

    for key, val in last_ts_person.items():
        # If last data received more than 1 one second then remove that person from Arena
        if ((t_now - val) > 1):
            refactorDraw( key, "leftHead", [], "#280B68", 'delete')
            refactorDraw( key, "rightHead", [], "#740C66", 'delete')
            refactorDraw( key, "torso", [], "#790A07", 'delete')
            refactorDraw( key, "leftArm", [], "#429A1D", 'delete')
            refactorDraw( key, "rightArm", [], "#818525", 'delete')
            refactorDraw( key, "leftLeg", [], "#113887", 'delete')
            refactorDraw( key, "rightLeg", [], "#1B9E92", 'delete')
            refactorDraw( key, "leftFoot", [], "#FFFFFF", 'delete')#0A0769"
            refactorDraw( key, "rightFoot", [], "#FFFFFF", 'delete') #"#0A0769"
            drawPersonText(key,"", [0, 0, 0], "#FFFFFF", action='delete')
            last_ts_person[key] = 0


def refactorDraw(person,bodypart,arr, color, action='create'):
    # partID
    partID = person+'_'+bodypart

    if (last_ts_person[person] != 0):
        action = 'update'

    # Generate message
    message = {}
    message['object_id'] = 'thickline_' + partID
    message['action'] = action
    message['type'] = 'object'

    # TODO: Check if body part is 0 (should not draw that part)
    # If action is create or update generate data part of the message
    if ((action == 'create') or (action == 'update')):
        data_str = ""
        # Iterate through each body part
        for i in range(len(arr)):
            if (arr[i][0] != 0) and (arr[i][1] != 0) and (arr[i][2] != 0):
                x = -arr[i][0]
                y = 2-arr[i][1]
                z = arr[i][2]
                data_str += str(x) + " " + str(y) + " " + str(z) + ", "
        data_str = data_str[:-2] # remove last comma

        data = {}
        data['object_type'] = 'thickline'
        data['lineWidth'] = 11
        data['color'] = color # '#FF88EE'
        data['path'] =  data_str # '0 0 0, 1 0 0, 1 1 0, 1 1 1'

        data_cont = {}
        data_cont['meshline'] = data

        message['data'] = data_cont

    # Generate JSON
    json_data = json.dumps(message)

    # Publish message
    new_draw_path = "realm/s/" + scene + "/"
    client.publish(new_draw_path + message['object_id'], json_data)

    # debugging
    # print(new_draw_path + message['object_id'], json_data)

def drawPersonText(person,bodypart, arr, color, action='create'):
    # partID
    partID = person

    # Generate message
    message = {}
    message['object_id'] = 'text_' + partID
    message['action'] = action

    # If action is create or update generate data part of the message
    data = {}
    data['object_type'] = 'text'
    data['color'] = color # '#FF88EE'
    data['text'] = person

    # import ipdb; ipdb.set_trace()

    position = {}
    position['x'] = -arr[0]
    position['y'] = 2.3-arr[1]
    position['z'] = arr[2]
    data['position'] = position


    rotation = {}
    rotation['x'] = 0
    rotation['y'] = 0
    rotation['z'] = 0
    rotation['w'] = 1
    data['rotation'] = rotation

    scale = {}
    scale['x'] = 0.5
    scale['y'] = 0.5
    scale['z'] = 0.5
    data['scale'] = scale


    message['data'] = data

    # Generate JSON
    json_data = json.dumps(message)

    # Publish message
    new_draw_path = "realm/s/" + scene + "/"
    client.publish(new_draw_path + message['object_id'], json_data)

    # print(json_data)


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
    print("Connected with result code " + str(rc))
    client.subscribe(TOPIC_3D) #subscribe
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
        theMessage=str(message.payload.decode("utf-8"))
        splits=theMessage.split(',')

        Person = splits[0]
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

        refactorDraw( Person, "leftHead", [ Nose, LEye, LEar ], "#280B68" )
        refactorDraw( Person, "rightHead", [ Nose, REye, REar ], "#740C66" )
        refactorDraw( Person, "torso", [ Nose, Neck, MidHip ], "#790A07" )
        refactorDraw( Person, "leftArm", [ Neck, LShoulder, LElbow, LWrist ], "#429A1D" )
        refactorDraw( Person, "rightArm", [ Neck, RShoulder, RElbow, RWrist ], "#818525" )
        refactorDraw( Person, "leftLeg", [ MidHip, LHip, LKnee, LAnkle ], "#113887" )
        refactorDraw( Person, "rightLeg", [ MidHip, RHip, RKnee, RAnkle ], "#1B9E92" )
        refactorDraw( Person, "leftFoot", [ LHeel, LAnkle, LBigToe, LSmallToe ], "#FFFFFF" ) #"#0A0769"
        refactorDraw( Person, "rightFoot", [ RHeel, RAnkle, RBigToe, RSmallToe ], "#FFFFFF" ) #"#0A0769"

        drawPersonText(Person, "name", Neck, "red")
        last_ts_person[Person] = time.time()

################## MQTT Init ##################
client = paho.Client()
client.on_connect=on_connect
client.on_message=on_message

################## MQTT Connect ##################
print("Connecting to broker: ", SERVER_ADDR)
client.connect(SERVER_ADDR)
client.loop_start() # start loop to process received messages

while True:
    if ((time.time() - last_ts_check) > 1):
        refreshArena()
    time.sleep(0.1)

client.disconnect() #disconnect
client.loop_stop() #stop loop