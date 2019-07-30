add_executable(posefusion-client 	posefusion-client.cpp)
add_executable(posefusion-server 	posefusion-server.cpp)
add_executable(camera-calibration 	camera-calibration.cpp)
add_executable(image-grab 			image-grab.cpp)

target_link_libraries(posefusion-client 
	openpose
	${examples_3rdparty_libraries}
	paho-mqttpp3
)

target_link_libraries(posefusion-server
	${examples_3rdparty_libraries}
	paho-mqttpp3
)

target_link_libraries(camera-calibration 
	${examples_3rdparty_libraries}
)

target_link_libraries(image-grab
	${examples_3rdparty_libraries}
)