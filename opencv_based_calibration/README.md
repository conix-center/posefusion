PoseFusion
====================================

Calibration Instructions

==============================

g++ fisheye_camera_calibration.cpp `pkg-config opencv --cflags --libs` -o fisheye_camera_calibration

./fisheye_camera_calibration in_VID5_fisheye.xml 

./build/examples/calibration/calibration.bin --mode 1 --grid_square_size_mm 114.0 --grid_number_inner_corners "8x6" --camera_serial_number logitech_brio --calibration_image_dir ./posefusion/calibration_images/


