import cv2
import numpy as np
import sys
import json
import paho.mqtt.client as mqtt
import time
import pandas as pd
import ipdb
from time import sleep

import matplotlib.pyplot as plt



# Global parameters
SERVER_ADDR     = "oz.andrew.cmu.edu"
TOPIC_3D        = "/topic/skeleton"
TOPIC_POSE      = "/lambda/+/pose"
TOPIC_POSE1      = "/lambda/1/pose"
TOPIC_POSE2      = "/lambda/2/pose"
TOPIC_POSE3      = "/lambda/3/pose"




# ipdb.set_trace()

######################### MQTT Functions #########################
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(TOPIC_POSE)

if __name__ == '__main__':

    path = "../results/"
    file = sys.argv[1]
    print(file)

    # ipdb.set_trace()
    lambda1 = open(path + file + "_l1", "r")
    lambda2 = open(path + file + "_l2", "r")
    lambda3 = open(path + file + "_l3", "r")

    lines_l1 = lambda1.readlines()
    lines_l2 = lambda2.readlines()
    lines_l3 = lambda3.readlines()

    # min_l = min(len(lines_l1), len(lines_l2))

    # diff_ts = []
    # for i in range(min_l):
    #     ts1 = int(lines_l1[i].split(" ")[0])
    #     ts2 = int(lines_l2[i].split(" ")[0])
    #     diff_ts.append(ts1-ts2)


    # fig = plt.figure()
    # ax = plt.axes()
    # ax.plot(list(range(min_l)), diff_ts);

    # # ax.plot(list(range(min_l)), ts_1[:min_l], ts_2[:min_l]);
    
    # plt.show()

    # ipdb.set_trace()


    ################## MQTT Init ################## 
    client = mqtt.Client()
    client.on_connect = on_connect
    # client.on_message = on_message

    ################## MQTT Connect ################## 
    client.connect(SERVER_ADDR, port=1883, keepalive=60)

    # ipdb.set_trace()

    total_l1 = 0
    for i in range(min(len(lines_l1), len(lines_l2), len(lines_l3))):
        data_string = []
        # if (len(lines_l1[i].split(" ")) == 54) and (len(lines_l2[i].split(" ")) == 54):
        ts1 = int(lines_l1[i].split(" ")[0])
        ts2 = int(lines_l2[i].split(" ")[0])
        ts3 = int(lines_l3[i].split(" ")[0])
        print(i, ts2-ts1)
        # ipdb.set_trace()
        client.publish(TOPIC_POSE1, lines_l1[i][:-3] + ",") 
        client.publish(TOPIC_POSE2, lines_l2[i][:-3] + ",") 
        client.publish(TOPIC_POSE3, lines_l3[i][:-3] + ",")
        sleep(0.03)