PoseFusion
====================================

## Installing PoseFusion
Run the install_posefushion.sh file and follow the on-screen command line instructions

Note: Installation file intended for Lambda Systems with Ubuntu OS

Also to see the mqtt published messages install :
sudo apt-get install mosquitto-clients

and run :
mosquitto_sub -v -t realm/s/# -h oz.andrew.cmu.edu
mosquitto_sub -v -t realm/s/# -h oz.andrew.cmu.edu | grep -v cube

