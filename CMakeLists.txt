add_executable(posefusion-client 	posefusion-client.cpp)
add_executable(posefusion-server 	posefusion-server.cpp)
add_executable(camera-calibration 	camera-calibration.cpp)
add_executable(image-grab 			image-grab.cpp)

target_link_libraries(posefusion-client 
	openpose
	${examples_3rdparty_libraries}
	paho-mqtt3cs
	paho-mqtt3as
        paho-mqttpp3

)

target_link_libraries(posefusion-server
        paho-mqtt3cs
        paho-mqtt3a
	paho-mqtt3as
	paho-mqttpp3
	openpose
	${examples_3rdparty_libraries}
)

target_link_libraries(camera-calibration 
	${examples_3rdparty_libraries}
)

target_link_libraries(image-grab
	${examples_3rdparty_libraries}
)

add_subdirectory(opencv_based_calibration)