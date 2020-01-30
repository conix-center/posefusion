PoseFusion
====================================

Calibration Instructions

==============================

g++ fisheye_camera_calibration.cpp `pkg-config opencv --cflags --libs` -o fisheye_camera_calibration

./fisheye_camera_calibration in_VID5_fisheye.xml 

./build/examples/calibration/calibration.bin --mode 1 --grid_square_size_mm 114.0 --grid_number_inner_corners "8x6" --camera_serial_number logitech_brio --calibration_image_dir ./posefusion/calibration_images/

./build/examples/openpose/openpose.bin --num_gpu 0 --camera 2 --write_images ./posefusion/fisheyeCalibration_images/


./build/examples/calibration/fisheyeCalibration.bin --mode 1 --grid_square_size_mm 114.0 --grid_number_inner_corners "8x6" --camera_serial_number fisheye_cam --calibration_image_dir ./posefusion/fisheyeCalibration_images/


====================================

Fisheye Camera Intrinsics :

Intrinsics:

Re-projection error - cv::calibrateCamera vs. fisheyeCalcReprojectionErrors:	62998834895943716123649167988829111976651640114977080215963489635957473280.000000 vs. inf

Intrinsics_K:
[-149390.8758233938, 1.700981741370965e+74, 4.686916689497339e+63;
 0, 1.623923680888597e+69, 1.775898752762208e+68;
 0, 0, 1]

Intrinsics_distCoeff:
[0.3814478301545055;
 -0.5882337345616587;
 0.5294893504797997;
 -0.1638780151720456]





