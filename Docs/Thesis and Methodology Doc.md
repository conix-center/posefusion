PoseFusion
====================================



## Purpose
This program reconstructs multiple OpenPose data points into a singular 3D pose in real time. The 3D coordinates are then published to a Unity application via MQTT, where the data can be eventually rendered into photorealistic avatar.


## Setup
The setup consists of 4 cameras in pairs of 2 forming 2 stereo pairs set-1 & set-2.


## Methodology
The methodology of Multi-Person 3D Pose estimation is as follows: 
1. Obtain 2D Pose points by running OpenPose on each camera
2. Obtain 3D Pose points using the 2D pose points of the stereo pair. As we have 2 stereo pairs we will get 2 estimates of 3D pose points   
'''
There are 2 methods currently used for estimating the 3D pose from 2D pose estimates:
a) Essential Matrix Mode : Using the OpenCV triangulatePoints() function. The 3D point is found from the equation that x = CX where x is 2D image point, X is the 3D world point, C is camera projection matrix (3D to 2D conversion matrix) and the fact that x cross-profuct with PX must be zero, so can write the equations from these into a linear form as AX = 0, and each camera in the stereo pair gives 2 equations thus A is 4x4 
    Sub-note : all points are in homogenious coordinate in the above equations
    
    pts4D = cv2.triangulatePoints(camera_1, camera_2, pt1, pt2).T

b) Stereo Mode : Using the concept of 'Depth from Stereo' based on human binocular vision system which calculates the disparity along the y-axis between the stereo camera pairs which identically oriented and positioned expect for a translation in the y-axis by a predefined amount called the baseline (distance between the optical centres of the 2 stereo camera pairs). From the calculated disparity, depth is estimated using the formula d = fx\*baseline/disparity\*unit, where fx is the focal length of the camera in x-axis and unit is the depth unit. 
    Sub-note : It is assumed fx is same for both the cameras here

    Ref : https://github.com/IntelRealSense/librealsense/blob/master/doc/depth-from-stereo.md
'''
3. Stereo set-1 is treated as the Global co-ordinate system, so Transformation matrix to translate stereo set-2 points to set-1 coordinate points is found. This is used to transform the set-2 3D pose estimate into the coordinate frame of set-1
4. Eliminate same human detection between the 2 stereo pairs by computing the SSD between the 3D pose estimate by stereo set-1 & set-2. If the error is less than the predefined threshold eliminate the 3D pose estimate by stereo set-2 as set-1 is preffered
5. Render all the 3D pose estimates of stereo set-1 & set-2 in the Arena

## TODO List
- [x] Understand the current methodology 
- [ ] Understand how the transformation matrix between stereo set-1 & set-2 is calculated
- [ ] Optimise the methodology
- [ ] Optimise the 3D pose estimation using 2D pose estimates
- [ ] Understand how the camera Projection, Intrinsic and Extrinsic matrices calculated
- [ ] TODO - GRG Ciritcal: How are we assuring the one-to-one body correspondance between the 2 stereo sets? It may so happen that body-1 in stereo set-1 may be actually body-2 in stereo set-2 and not body-1
        

## Reference Articles 
1. https://github.com/IntelRealSense/librealsense/blob/master/doc/depth-from-stereo.md
2. http://nghiaho.com/?page_id=671
3. https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
4. https://github.com/CMU-Perceptual-Computing-Lab/openpose 