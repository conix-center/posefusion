#!/bin/bash

echo "Started setup to install posefusion"

mkdir PoseFusion
cd PoseFusion

sudo apt-get update

echo "Started Installing Git"
sudo apt-get install git
echo "Git installation finished"

sudo apt-get install python-pip

echo "Started Installing CMake"
sudo apt remove --purge --auto-remove cmake
sudo snap install --classic cmake 

#mkdir tmp
#cd tmp
#version = 3.16
#build = 2
#sudo wget https://github.com/Kitware/CMake/releases/download/v$version.$build/cmake-$version.$build-Linux-x86_64.sh
#sudo mkdir /opt/cmake
#sudo sh cmake-$version.$build-Linux-x86_64.sh --prefix=/opt/cmake
#sudo rm /usr/local/bin/cmake 
#sudo ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake
#cd ../
#sudo apt-get -y install cmake-qt-gui
#sudo apt-get install build-essential gcc make cmake cmake-gui cmake-curses-gui
#sudo apt-get install libssl-dev 
#sudo apt-get install doxygen graphviz

echo "CMake Installation finished"

echo "Started installing CUDA"
sudo apt-get update && sudo apt-get install -y lambda-stack-cuda
echo "Cuda installation finished"

#echo "Started installing openCV"
#mkdir openCV
#cd openCV

#sudo apt-get install build-essential
#sudo apt-get install libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

#git clone https://github.com/opencv/opencv.git
#cd opencv
#git checkout 3.4
#cd ..

#wget https://github.com/opencv/opencv/archive/3.2.0.tar.gz
#mkdir opencv && tar -zxvf 3.2.0.tar.gz -C opencv --strip-components 1

#wget https://github.com/opencv/opencv_contrib/archive/3.2.0.tar.gz
#mkdir opencv_contrib && tar -zxvf 3.2.0.tar.gz -C opencv_contrib --strip-components 1

#git clone https://github.com/opencv/opencv_contrib.git
#cd opencv_contrib
#git checkout 3.4
#cd ..

#echo "Complete Configuration and Generation on CMake and press any key to continue installation"
#echo "Note : Set OPENCV_EXTRA_MODULES_PATH: to the PoseFusion/openCV/opencv_contrib/modules"
#echo "Note : Due to a current bug in CUDA disable cudacodec option. For more info see OpenCV_contrib issue 1786"
#sudo cmake-gui

#Wait till user presses any key
#read  -n 1 -p "Continue:" keypress

#cd opencv/build/
#sudo make -j`nproc`

#Required to include openpose cpp as library in global enviornment file
#sudo make install

#cd ../..
#echo "openCV Installation finished"

echo "Started installing MQTT"
#For Python - MQTT python version
pip3 install paho-mqtt python-etcd

#MQTT Dependencies
sudo apt-get install libssl-dev 
sudo apt-get install doxygen graphviz

#Install Paho-Mqtt-C
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
git checkout v1.3.1

sudo cmake -Bbuild -H. -DPAHO_WITH_SSL=ON -DPAHO_ENABLE_TESTING=OFF
sudo cmake --build build/ --target install
sudo ldconfig

#Install Paho-Mqtt-Cpp
git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp
cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
sudo cmake --build build/ --target install
sudo ldconfig
cd ../
echo "MQTT installation finished"

echo "Installing OpenPose dependencies"
sudo apt-get install libprotobuf-dev
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libopencv-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libhdf5-serial-dev
sudo apt-get install libatlas-base-dev
echo "OpenPose dependencies installation finished"

echo "Started installing openpose"
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose

cd openpose/3rdparty/
git clone https://github.com/CMU-Perceptual-Computing-Lab/caffe.git
cd ../

echo "Complete Configuration and Generation on CMake and press any key to continue installation"
sudo cmake-gui

#Wait till user presses any key
read  -n 1 -p "Continue:" keypress

cd build/
sudo make -j`nproc`

#Required to include openpose cpp as library in global enviornment file
sudo make install

#Added PoseFusion dependency in CMake of openpose
echo '#Added PoseFusion dependency' >> CMakeLists.txt
echo 'add_subdirectory(posefusion)' >> CMakeLists.txt
cd ..
echo "Openpose installation finished"

echo "Clonning posefusion"

git clone https://github.com/conix-center/posefusion.git

#TODO - GRG - Note : checkingout posefusion_2camera_setup for now 
# as it as the valid code for compilation which needs to be moved to master branch later
cd posefusion
git checkout posefusion_2camera_setup
cd ..

echo "Please enter the lambda device id:"
read  -n 1 -p "Lambda Id:" lambdaId

#sed -i -e 's/.*static std::string CLIENT_ID.*/static std::string CLIENT_ID   = "lambda-$lambdaId"; // "lambda-3";/' posefusion/posefusion-client.cpp
sed -i -e "s/.*static std::string CLIENT_ID.*/static std::string CLIENT_ID   = \"lambda-$lambdaId\"; \/\/ \"lambda-3\";/" posefusion/posefusion-client.cpp

echo "Complete Configuration and Generation on CMake and press any key to continue installation"
sudo cmake-gui

#Wait till user presses any key
read  -n 1 -p "Continue:" keypress

cd build/
sudo make -j`nproc`
cd ../..

echo "Creating python virtual enviornment"
sudo apt-get install python3-venv
# sudo apt update
# sudo apt install virtualenv
# mkdir python-environments
# cd python-environments

# virtualenv -p python3 pyenv-posefusion
python3 -m venv pyenv-posefusion

#Activate Python Enviornment
source pyenv-posefusion/bin/activate

echo "Installing required python libraries"
#sudo apt-get install python-pip

#pip install --user --requirement requirements.txt
sudo pip install --user --requirement ../requirements.txt

#Deactivate Python Virtual Enviornment
deactivate

sudo chmod 755 posefusion/posefusion-*

echo "Python virtual enviornment created"

echo "Posefusion installation completed!"
echo "Done!!!"
