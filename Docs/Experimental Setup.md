PoseFusion
====================================



## Experimental Setup 
For calibration and determining the error/accuracy of projection and 3D pose estimation, only a single stereo pair is used.


## Camera Setup
On facing the setero pair:
Camera-1 : Camera on the left side of the stereo pair when facing them
Camera-2 : Camera on the right side of the stereo pair when facing them
Above/Top : is positive y-axis
Right : is positive x-axis
Towards you : is positive z-axis

The camera-1 of the stereo pair is treated as world origin translated along y-axis by the height of the camera (y_offset). Therefore its world coordinates would be [x,y,z] = [0, y_offset, 0]. The second camera has the world coordintes [x,y,z] = [baseline, y_offset, 0] The cameras are at a height of about 70 inch (1.8m). The baseline is about 39.5inch (1m).
