cmake_minimum_required(VERSION 3.8)
project(staubli_tx2_90_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()