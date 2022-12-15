cmake_minimum_required(VERSION 2.8.3)
project(aliengo_description)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    std_msgs
    aliengo_kindym
    aliengo_msgs
    tf
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
add_executable(odometry src/odometry.cpp)
add_dependencies(odometry nav_msgs_generate_messages_cpp)
target_link_libraries(odometry   ${catkin_LIBRARIES})
add_executable(footstep_state_publisher src/footstep_current_state_node.cpp)
target_link_libraries(footstep_state_publisher   ${catkin_LIBRARIES})
add_dependencies(footstep_state_publisher aliengo_msgs_generate_messages_cpp)