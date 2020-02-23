# -*- coding: utf-8 -*-
"""
Created on Thu Oct 05 11:38:41 2017

@author:    Carlo Sferrazza
            PhD Candidate
            Institute for Dynamics Systems and Control
            ETH Zurich
            
Compares the calibration and undistortion methods of OpenCV and OCamLib
"""

import sys
import os
import pickle

import numpy as np


import cv2
import sys


""" Shows three parallel video streams (original, undistorted through opencv, 
                                        undistorted through Scaramuzza's toolbox) """
def undistortedStream(idx, mapx_persp_32, mapy_persp_32):
    cap = cv2.VideoCapture(idx)

    i = 0
    
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
    
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # undistort
        dst_scaramuzza = cv2.remap(gray, mapx_persp_32, mapy_persp_32, cv2.INTER_LINEAR)
        
        # Display the resulting frame
        cv2.imshow('undistorted scaramuzza', dst_scaramuzza)
        cv2.imshow('original', gray)

        cv2.imwrite(f'{i}.png', dst_scaramuzza)
    
        q = cv2.waitKey(1)
    
        if q & 0xFF == ord('q'):
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
""" Acquires images and returns them for calibration """
def acquireImages(idx):
    cap = cv2.VideoCapture(idx)
    images = []
    count = 0
    
    print("Press t for acquiring a new image, q to stop acquisition and proceed to calibration. "\
    "Try to collect at least 15 images from different viewpoints, and as close as possible "\
    "to the chessboard")
    sys.stdout.flush()
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
    
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        # Display the resulting frame
        cv2.imshow('frame',gray)
    
        q = cv2.waitKey(1)
    
        if q & 0xFF == ord('t'):
            images.append(gray)
            print("Images acquired: ", count)
            sys.stdout.flush()
            count+=1
        elif q & 0xFF == ord('q'):
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
    return images
    


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
    
def cam2world(point2D, o):
    point3D = []
    
    invdet = 1.0/(o['c']-o['d']*o['e'])

    xp = invdet*((point2D[0]-o['xc']) - o['d']*(point2D[1]-o['yc']))
    yp = invdet*(-o['e']*(point2D[0]-o['xc']) + o['c']*(point2D[1]-o['yc']))
    
    r = np.linalg.norm([xp,yp])
    
    zp = o['pol'][0]
    r_i = 1.0
    
    for i in range(1,o['length_pol']):
        r_i *= r
        zp += r_i*o['pol'][i]
        
    invnorm = 1.0/np.linalg.norm([xp,yp,zp])
    
    point3D.append(invnorm*xp)
    point3D.append(invnorm*yp)
    point3D.append(invnorm*zp)
    
    return point3D

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
        
def create_panoramic_undistortion_LUT(Rmin, Rmax, o):

    mapx = np.zeros((o['height'],o['width']))    
    mapy = np.zeros((o['height'],o['width']))    
    
    for i in range(o['height']):
        for j in range(o['width']):
            theta = -(float(j))/o['width']*2*np.pi
            rho = Rmax - float(Rmax-Rmin)/o['height']*i
            mapx[i,j] = o['yc'] + rho*np.sin(theta)
            mapy[i,j] = o['xc'] + rho*np.cos(theta)
            
    return mapx, mapy
          

if __name__ == '__main__':
    # Change here the index that corresponds to your camera (on Windows, add 700 to use DSHOW, which is generally faster)
    idx = 0 # + 700

    # Change here the path of the calibration file that contains the parameters obtained through ocam_calibration.py
    path_ocam = "./grg_ocam_calibration.txt"

    # Change here the number of internal corners that have to be detected on the chessboard in each dimension
    m = 9
    n = 6

    # Parameter that affect the result of Scaramuzza's undistortion. Try to change it to see how it affects the result
    sf = 4.0


        
    o = get_ocam_model(path_ocam)
    mapx_persp, mapy_persp = create_perspective_undistortion_LUT(o, sf)

    mapx_persp_32 = mapx_persp.astype('float32')
    mapy_persp_32 = mapy_persp.astype('float32')

    undistortedStream(idx, mapx_persp_32, mapy_persp_32)