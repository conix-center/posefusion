PoseFusion
====================================



## Purpose
This program reconstructs multiple OpenPose data points into a singular (currently only works for 1 person) 3D pose in real time. The 3D coordinates are then published to a Unity application via MQTT, where the data can be eventually rendered into photorealistic avatar.


## Dependencies
Including the dependencies that OpenPose requires, this program also uses the [Paho MQTT C++](https://github.com/eclipse/paho.mqtt.cpp) Library. Follow their instructions to download and install MQTT.


## Installing and Compiling
1. Install/compile [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose).
2. Clone **posefusion** into the openpose folder on your computer.
3. In the file openpose/CMakeLists.txt, write `add_subdirectory(posefusion)`. This should include posefusion in the cmake build.
4. Compile OpenPose.
```
cd build/
make -j`nproc`
```


## Running the Program

**PoseFusion Client**
Must be run in the **openpose** folder.
```
./build/posefusion/posefusion-client <lambda_id>
```
This runs the openpose program that gets body pose data from a single RGB camera. The pose then gets published to a MQTT broker. Each computer must be running the client in order for the PoseFusion server to run properly. <lambda_id> can be {1, 2, ...}

**PoseFusion Server**
```
python3 ./posefusion/python/posefusion.py
```
This subscribes to the pose data topics and reads in the constantly updating body pose. It then applies the camera matrices stored from the most recent camera calibration to the pose estimates and reconstructs a 3D version of openpose. The resulting 3D data then gets sents to the ARENA via MQTT to be rendered.

Several parameters are available to configure the posefusion server:

*General configuation*
 - MAX_NUM_PEOPLE: the maximum number of people supported (default: 20).
 - NUM_CAMERAS: the number of cameras used (default: 3).
 - REF_CAM: the reference camera used for reconstrution (default: 2).
 - CONF_SCORE: the minimum confidence score sent by openpose to be considered a valid person (default: 0.7)

*Calibration*
 - LOAD_PROJS: if sets to true then the projections are loaded from PROJS_PATH otherwise the auto calibration algorithm will
               be ran first (default: True).
 - MIN_BODY_CALIB: the minimum number of bodies that the program needs to collect before running the algorithm to find the
                   projection matrices (default: 500)

*Reconstruction*
 - ERR_THRESHOLD: the maximum reprojection error possible to consider the reconstructed 3D body valid (default: 10000).
 - FRAME_AVERAGING: the number of frames to be averaged before reconstruction (default: 1)
 - MIN_BODY_POINTS: the minimum number of pody points (out of 25*parts * 3*values = 75) that pass the validity filter required before sending it to the ARENA (default: 50).

**Display skeletons Arena**
```
python3 ./posefusion/python/skel.py
```
This scripts publishes the skeletons in the /topic/skeleton to the ARENA using thickline objects.

## Previous system
**Camera Calibration:**
On each computer, run 
```
./build/posefusion/image-grab
``` 
Make sure that the checkerboard is in each frame, and press 'q' on each image to save the calibration image.
Then run the calibration script.
```
./posefusion/calibration.sh
```
This scp the corresponding calibration images from each lambda machine and run camera-calibration.cpp. This takes in N images of a checkerboard from different angles and computes the camera matrices required for triangulation. The matrices will be saved in *openpose/media/matrices.xml* for use later on. When facing the calibration checkerboard, the cameras should be in ascending order from left to right (Left most is camera 1, right most is camera N).

**PoseFusion Server**
```
./build/posefusion/posefusion-server
```
This subscribes to the pose data topics and reads in the constantly updating body pose. It then applies the camera matrices stored from the most recent camera calibration to the pose estimates and reconstructs a 3D version of openpose. The resulting 3D data then gets sents to Unity via MQTT to be rendered.


## TODO List
- [x] Finish documenting code
- [x] Include multi-person functionality (right now only works with one person in frame)
- [x] Include filtering to determine if the reconstructed point is realistic in relation to proportions of a human
- [x] Make an automatic distributed camera calibration system instead of having to transfer images