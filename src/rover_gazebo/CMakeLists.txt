cmake_minimum_required(VERSION 3.8)
project(rover_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS rover_gazebo/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
