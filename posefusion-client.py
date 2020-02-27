# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
from sys import platform
import argparse
import numpy as np

import time


def initOpenpose():
    # Change these variables to point to the correct folder (Release/x64 etc.)
    sys.path.append('../build/python/');
    # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
    # sys.path.append('/usr/local/python')
    from openpose import pyopenpose as op
        

    # Flags
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_path", default="../examples/media/COCO_val2014_000000000192.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
    args = parser.parse_known_args()

    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = "../models/"

    # Add others in path?
    for i in range(0, len(args[1])):
        curr_item = args[1][i]
        if i != len(args[1])-1: next_item = args[1][i+1]
        else: next_item = "1"
        if "--" in curr_item and "--" in next_item:
            key = curr_item.replace('-','')
            if key not in params:  params[key] = "1"
        elif "--" in curr_item and "--" not in next_item:
            key = curr_item.replace('-','')
            if key not in params: params[key] = next_item

    # Construct it from system arguments
    # op.init_argv(args[1])
    # oppython = op.OpenposePython()

    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

   

    return opWrapper, op

""" Reads file containing the ocamcalib parameters exported from the Matlab toolbox """
def get_ocam_model(filename):
    o = {}
    with open(filename) as f:
        lines = [l for l in f]
        
        l = lines[2]
        data = l.split()
        o['length_pol'] = int(data[0])
        o['pol'] = [float(d) for d in data[1:]]
        
        l = lines[6]
        data = l.split()
        o['length_invpol'] = int(data[0])
        o['invpol'] = [float(d) for d in data[1:]]
        
        l = lines[10]
        data = l.split()
        o['xc'] = float(data[0])
        o['yc'] = float(data[1])
        
        l = lines[14]
        data = l.split()
        o['c'] = float(data[0])
        o['d'] = float(data[1])
        o['e'] = float(data[2])
                
        l = lines[18]
        data = l.split()
        o['height'] = int(data[0])
        o['width'] = int(data[1])

    return o
    
def world2cam(point3D, o):
    point2D = []    
    
    norm = np.linalg.norm(point3D[:2])

    if norm != 0:
        theta = np.arctan(point3D[2]/norm)
        invnorm = 1.0/norm
        t = theta
        rho = o['invpol'][0]
        t_i = 1.0
        
        for i in range(1,o['length_invpol']):
            t_i *= t
            rho += t_i*o['invpol'][i]
            
        x = point3D[0]*invnorm*rho
        y = point3D[1]*invnorm*rho
         
        point2D.append(x*o['c']+y*o['d']+o['xc'])
        point2D.append(x*o['e']+y+o['yc'])
    else:
        point2D.append(o['xc'])
        point2D.append(o['yc'])
        
    return point2D

def create_perspective_undistortion_LUT(o, sf):

    mapx = np.zeros((o['height'],o['width']))    
    mapy = np.zeros((o['height'],o['width']))    
    
    Nxc = o['height']/2.0
    Nyc = o['width']/2.0   
    Nz = -o['width']/sf       

    for i in range(o['height']):
        for j in range(o['width']):
            M = []
            M.append(i - Nxc)
            M.append(j - Nyc)
            M.append(Nz)
            m = world2cam(M, o)     
            mapx[i,j] = m[1]
            mapy[i,j] = m[0]
            
    return mapx, mapy


if __name__ == "__main__":

    camera_idx = 0

    recal_intrinsic = True

    path_ocam = "python/grg_ocam_calibration_480x640.txt"

    # Change here the number of internal corners that have to be detected on the chessboard in each dimension
    m = 9
    n = 6

    # Parameter that affect the result of Scaramuzza's undistortion. Try to change it to see how it affects the result
    sf = 4.0

    if recal_intrinsic:  
        o = get_ocam_model(path_ocam)
        mapx_persp, mapy_persp = create_perspective_undistortion_LUT(o, sf)

        mapx_persp_32 = mapx_persp.astype('float32')
        mapy_persp_32 = mapy_persp.astype('float32')
        np.save('mapx_persp_32_sm.npy', mapx_persp_32)
        np.save('mapy_persp_32_sm.npy', mapy_persp_32)

        np.savetxt('mapx_persp_32_sm.csv', np.pad(mapx_persp_32, ((0,0),(0,1))), delimiter=',', comments='')
        np.savetxt('mapy_persp_32_sm.csv', np.pad(mapy_persp_32, ((0,0),(0,1))), delimiter=',', comments='')
    else:
        mapx_persp_32 = np.load('mapx_persp_32.npy')
        mapy_persp_32 = np.load('mapy_persp_32.npy')


    opWrapper, op = initOpenpose()
    datum = op.Datum()

    cap = cv2.VideoCapture(camera_idx)

    # Set Resolution
    cap.set(3, 1280)
    cap.set(4, 720)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # frame = cv2.imread('fisheye_calibration_images2/Right_A16.png')
        print(f'frame shape {frame.shape}')
        
        # Our operations on the frame come here
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # undistort
        dst_scaramuzza = cv2.remap(frame, mapx_persp_32, mapy_persp_32, cv2.INTER_LINEAR)

         # Process Image
        # datum = op.Datum()
        datum.cvInputData = dst_scaramuzza

        # opWrapper.emplaceAndPop([datum])
        opWrapper.waitAndEmplace([datum])

        # Display Image
        print("Body keypoints: \n" + str(datum.poseKeypoints))

        # import matplotlib.pyplot as plt
        # plt.imshow(datum.cvOutputData)
        # plt.show()
        

        print(f'datum shape {datum.cvOutputData.shape}')
        
        # if datum.cvOutputData.shape != (0,0):
        #     cv2.imshow("Posefusion Client", datum.cvOutputData)
        #     # cv2.imshow("Posefusion Client", frame)
            
        #     key = cv2.waitKey(1)

        #     if key == ord('q'):
        #         cv2.destroyAllWindows()
        #         cap.release()
        #         break
        # else:
        #     print('datum output null')
        #     cv2.destroyAllWindows()
        #     cap.release()

        cv2.imshow("Posefusion Client", dst_scaramuzza)
        # cv2.imshow("Posefusion Client", datum.cvOutputData)
        # cv2.imshow("Posefusion Client", frame)
        
        key = cv2.waitKey(10)

        if key == ord('q'):
            cv2.destroyAllWindows()
            cap.release()
            break

        # time.sleep(5)

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

    opWrapper.stop()