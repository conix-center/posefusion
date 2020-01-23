add_executable(camera_calibration	camera_calibration.cpp)
add_executable(fisheye_camera_calibration	fisheye_camera_calibration.cpp)

target_link_libraries(camera_calibration 
	${examples_3rdparty_libraries}
)

target_link_libraries(fisheye_camera_calibration 
	${examples_3rdparty_libraries}
)