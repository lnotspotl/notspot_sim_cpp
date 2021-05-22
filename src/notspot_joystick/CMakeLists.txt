cmake_minimum_required(VERSION 3.0.2)
project(notspot_joystick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES notspot_joystick
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ramped_joystick 
    src/main.cpp 
    src/PS4_controller.cpp
    )
target_link_libraries(ramped_joystick ${catkin_LIBRARIES})
