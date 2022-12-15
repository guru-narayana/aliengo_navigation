cmake_minimum_required(VERSION 3.0.2)
project(aliengo_footstep_planner)


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
# )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_msgs
  grid_map_ros
  geometry_msgs
  sensor_msgs
)



catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
)


add_executable(quad_footstep_planner_node src/quad_footstep_planner_node.cpp src/quad_footstep_planner.cpp)
target_link_libraries(quad_footstep_planner_node ${catkin_LIBRARIES} )
add_dependencies(quad_footstep_planner_node aliengo_msgs_generate_messages_cpp)