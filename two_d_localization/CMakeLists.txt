cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(two_d_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES filling_extraction
CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(WheelOdometry
  src/WheelOdometry.cpp)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp src/WheelOdometry.cpp)
target_link_libraries(wheel_odometry_node ${catkin_LIBRARIES})


