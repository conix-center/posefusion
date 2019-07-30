#!/bin/bash

# for i in 1 2 3 4
for i in 1 3 4		# only using lambdas 1 3 and 4 currently
do
	scp lambda-$i:/home/wiselab/openpose/media/calib$i.jpg /home/wiselab/openpose/media/
done

./build/posefusion/camera-calibration ./media/calib4.jpg ./media/calib3.jpg ./media/calib1.jpg
