add_executable(posefusion posefusion.cpp)
add_executable(posefusion-server posefusion-server.cpp)

target_link_libraries(posefusion 
  openpose
  ${examples_3rdparty_libraries}
  paho-mqttpp3
)

target_link_libraries(posefusion-server
  paho-mqttpp3
)