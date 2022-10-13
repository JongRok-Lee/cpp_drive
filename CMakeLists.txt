cmake_minimum_required(VERSION 3.0.2)
project(cpp_drive)

find_package(catkin REQUIRED COMPONENTS
  # cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  xycar_msgs
  #OpenCV
)
find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cpp_drive
 CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs xycar_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)

add_executable(drive src/main.cpp)

target_link_libraries(drive
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
