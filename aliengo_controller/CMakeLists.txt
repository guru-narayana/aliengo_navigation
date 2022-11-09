cmake_minimum_required(VERSION 3.0.2)
project(aliengo_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  aliengo_kindym
  geometry_msgs
  sensor_msgs
  aliengo_msgs
  tf
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES aliengo_controller
  CATKIN_DEPENDS roscpp rospy std_msgs aliengo_kindym
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(quad_controller_node src/controller.cpp src/shifting_control.cpp src/trot.cpp src/walk.cpp)
target_link_libraries(quad_controller_node ${catkin_LIBRARIES} )