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
Make sure that **ALL** of these programs are run in the **openpose** folder.

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

**PoseFusion Client**
```
./build/posefusion/posefusion-client
```
This runs the openpose program that gets body pose data from a single RGB camera. The pose then gets published to a MQTT broker. Each computer must be running the client in order for the PoseFusion server to run properly.

**PoseFusion Server**
```
./build/posefusion/posefusion-server
```
This subscribes to the pose data topics and reads in the constantly updating body pose. It then applies the camera matrices stored from the most recent camera calibration to the pose estimates and reconstructs a 3D version of openpose. The resulting 3D data then gets sents to Unity via MQTT to be rendered.


## TODO List
- [ ] Finish documenting code
- [ ] Include multi-person functionality (right now only works with one person in frame)
- [ ] Make an automatic distributed camera calibration system instead of having to transfer images
