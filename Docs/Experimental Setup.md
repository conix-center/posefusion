PoseFusion
====================================


Note : All units of length is meters. Arena also defines length in meters.

## Experimental Setup 
For calibration and determining the error/accuracy of projection and 3D pose estimation, only a single stereo pair is used.

Basline = 1m
Camera height = 1.8m (approx)

## Camera Setup
On facing the setero pair:
Camera-1 : Camera on the left side of the stereo pair when facing them
Camera-2 : Camera on the right side of the stereo pair when facing them
Above/Top : is positive y-axis
Right : is positive x-axis
Towards you : is positive z-axis

The camera-1 of the stereo pair is treated as world origin translated along y-axis by the height of the camera (y_offset). Therefore its world coordinates would be [x,y,z] = [0, y_offset, 0]. The second camera has the world coordintes [x,y,z] = [baseline, y_offset, 0] The cameras are at a height of about 70 inch (1.8m). The baseline is about 39.5inch (1m).

## Camera Matices

# Intrinsic Matrices
Since both the cameras used are same, the intrinsic matrix is the same (approximately) for both. It is calculated using the openCV calibration function employing a checkered board.

Checkere Board dimensions:
Width = 80cm (8m)
Height = 103cm (10.3m)
Square side length = 11.43cm (1.143m)

# External Matrices
With the perspective set as facing the camera front and with the same perspective in the arena. The arena co-ordinate system has positive x-axis along the right direction, positive y-axis towards the top and positive z-axis towards you i.e. away from the camera. Basically right handed system.

For the camera co-ordinates, the positive x-axis is along left direction, the positive y-axis is towards the bottom and positive z-axis is towards you i.e. away from the camera. Basically right handed system, but a different orientation then the arean.

So, the rotation matrix will need to flip x & y axis.
Therefore,

R = [-1, 0, 0
      0,-1, 0
      0, 0, 1]

Rotation matrix for both the cameras is the same, as we have oriented them in the same direction (approximately).

Translation matrix for camera-1 is [0, camera_height, 0]
Translation matrix for camera-2 is [baseline, camera_height, 0]

# Projection Matrix aka Camera matrix

The Projection Matrix aka Camera matrix maps the 3D world point to 2D image point. So is obtained by,

P = A[R|T]

where, P - Projection matrix
       A - Intrinsic matrix
       R - Rotation matrix
       T - Translation matrix


