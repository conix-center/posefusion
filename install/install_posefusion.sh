#!/bin/bash

echo "Started setup to install posefusion"

mkdir PoseFusion
cd PoseFusion

echo "Started Installing CMake"
#sudo apt-get -y install cmake-qt-gui
sudo apt-get install build-essential gcc make cmake cmake-gui cmake-curses-gui
sudo apt-get install libssl-dev 
#sudo apt-get install doxygen graphviz
echo "CMake Installation finished"

echo "Started installing MQTT"
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
git checkout v1.3.1

cmake -Bbuild -H. -DPAHO_WITH_SSL=ON -DPAHO_ENABLE_TESTING=OFF
sudo cmake --build build/ --target install
sudo ldconfig
echo "MQTT installation finished"

echo "Started installing openpose"
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose

echo "Complete Configuration and Generation on CMake and press any key to continue installation"
cmake-gui

#Wait till user presses any key
read  -n 1 -p "Input Selection:" mainmenuinput

cd build/
make -j`nproc`

echo "Openpose installation finished"

echo "Clonning posefusion"

git clone https://github.com/conix-center/posefusion.git

echo "Please enter the lambda device id:"
read  -n 1 -p "Input Selection:" lambdaId

sed -i -e 's/.*static std::string CLIENT_ID.*/static std::string CLIENT_ID   = "lambda-$lambdaId"; // "lambda-3";/' posefusion/posefusion-client.cpp


echo "Creating python virtual enviornment"

# sudo apt update
# sudo apt install virtualenv
# mkdir python-environments
# cd python-environments

# virtualenv -p python3 pyenv-posefusion
python3 -m venv pyenv-posefusion

#Activate Python Enviornment
source pyenv-posefusion/bin/activate

echo "Installing required python libraries"
pip install --user --requirement requirements.txt

#Deactivate Python Virtual Enviornment
pyenv-posefusion/bin/deactivate

echo "Python virtual enviornment created"

echo "Posefusion installation completed!"
echo "Done!!!"