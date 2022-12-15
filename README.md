# Aliengo Navigation Software stack

[![Repository Status](https://img.shields.io/badge/Repository%20Status-Maintained-dark%20green.svg)](https://github.com/guru-narayana/aliengo_navigation)
[![Author](https://img.shields.io/badge/Author-Nara%20Guru%20Narayanaswamy-blue)](https://www.linkedin.com/in/nara-guru-narayanaswamy-658a811b0/)

# Introduction
This work presents software stack for indoor Vision-based Quadrupedal Locomotion. It integrates existing technology in Simultaneous Localisation and Mapping (SLAM), global path planning, and composite COG Trajectory planning methods to achieve both autonomous and semi-autonomous Quadrupedal Locomotion. The work is currently tested in simulation with Unitree’s Alien go Robot and can be extended to reality. Unlike the existing SDK, this software presents all in one solution to locomotion problems, including vision-based foothold planning and obstacle avoidance. The modular architecture of the Software allows various controller implementations

# Dependencies
* [ROS](https://www.ros.org/)
* [Gazebo8](http://gazebosim.org/)
* [RTAB_Map](https://github.com/introlab/rtabmap_ros)
* [ros_control](http://wiki.ros.org/ros_control)
* [Navigation](https://github.com/ros-planning/navigation)
* [geometry2](https://github.com/ros/geometry2)
* [elevation mapping](https://github.com/ANYbotics/elevation_mapping)
* [realsense_gazebo_custom](https://github.com/guru-narayana/Aliengo_2D_Nav-sim/tree/main/realsense_gazebo_custom)
* [Joy](http://wiki.ros.org/joy)


# Building

In order to install the Aliengo Navigation Software stack, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/guru-narayana/aliengo_navigation
    cd ..
    catkin_make

# Detail of Packages

## unitree_gazebo:
You can launch the Gazebo simulation of the aliengo robot with the following command:
```
roslaunch unitree_gazebo aliengo_gazebo.launch
```

## aliengo_utils
To launch simulation and RTAB-Map at once use the following
```
roslaunch aliengo_utils  Aliengo_sim.launch
```

And you can use the joystick to plan the footholds by launching
```
roslaunch aliengo_utils foot_planner.py
```
This will only plan the footholds but will not execute them, inorder to execute the plan use the controller node.

# Nodes 
## Quad_controller_node
To launch the controller node you can use the following command.
```
rosrun aliengo_controller quad_controller_node 
```
# Params
All the parameters are located in the aliengo_utils package.
## Quad_footstep_planner:

* **`Grid_map_topic`** (string, default: "/elevation_mapping/elevation_map_raw")

    The name of the topic to which elevation map is being published
    
* **`Grid_map_resolution`** (double, default: 0.05)

    The resolution of the grid map

* **`visualize_plan`** (bool, default: true)

    Choose to visualise the planned footholds in the Rviz.

* **`max_steplength`** (double, default: 0.19)

    Maximum step length the robot can consider to avoid edges.

* **`min_steplength`** (double, default: 0.07)

    Minimum step length the robot can consider to avoid edges.

* **`favoured_steplength`** (double, default: 0.9)

    Step length favoured by the user, which is used to get natural motion and reduce sensitivity towards the noise.

* **`resolution_steplength`** (double, default: 0.01)

    Resolution of the step lenth search space that is used to avoid edges.

* **`prefered_stepcostFactor`** (double, default: 3.5)

    The higher the cost the more robot tries to plan the footholds with the favoured steplength.

* **`obstacle_stepcostFactor`** (double, default: 0.01)

    The higher the cost the more robot tries to plan the footholds in the region with less height.

* **`edge_costFactor`** (double, default: 30.0)

    The higher the cost the more robot tries to avoid edges.

* **`collision_costFactor`** (double, default: 1.2)

    The higher the cost the more robot tries to avoid the collisions happening in its horizon (Used only in fully autonomous mode).

* **`global_costFactor`** (double, default: 0.4)

    The higher the cost the more robot tries to follow the global path (Used only in fully autonomous mode).

* **`using_joystick`** (bool, default: true)

    Set it to true to control the robot using the joystick.

* **`joystick_topic`** (string, default: "/joy")

    The name of the topic to which joystick vals are being published.

* **`horizon_length`** (int, default: 4)

    The number of steps into the future the collision occurence needs to be checked.

* **`foot_plan_horizon`** (int, default: 5)

    The number of steps into the future the footholds will be planned.

* **`vx_samples`** (int, default: 5)

    The number of linear velocity samples to be used to plan the footholds for global path.

* **`vtheta_samples`** (int, default: 10)

    The number of angular velocity samples to be used to plan the footholds for global path.

* **`global_goal_topic`** (string, default: "/aliengo_goal")

    The name of the topic to which joystick vals are being published.

## Robot_config:

* **`base_length`** (double, default: 0.4798)

    The length of robot base.

* **`base_width`** (double, default: 0.102)

    The width of robot base.

* **`L1`** (double, default: 0.083)

    The length of link L1 of the robot.

* **`L2`** (double, default: 0.25)

    The length of link L2 of the robot.

* **`L3`** (double, default: 0.25)

    The length of link L3 of the robot.

* **`foot_radius`** (double, default: 0.0265)

    The radius of the foot sphere of the robot.

* **`collision_rect_length`** (double, default: 0.65)

    The length of collision checking polygon.

* **`collision_rect_width`** (double, default: 0.3)

    The width of collision checking polygon.

* **`collision_point_height`** (double, default: 0.2)

    The height of the grid cell above which it should be considered as a collision cell.

* **`collision_free_threshold`** (double, default: 99.5)
    
    Percentage of free space to be present in the collision polygon to not consider it as a possible collision.


* **`collision_threshold`** (double, default: 0.01)

    The collision cost above which the robot will halt.

* **`max_forward_vel`** (double, default: 0.18)

    The maximum forward velocity of the base of the robot.

* **`max_angular_vel`** (double, default: 0.1)

    The maximum angular velocity of the base of the robot.(anticlockwise vel)

* **`min_angular_vel`** (double, default: -0.1)

    The minimum angular velocity of the base of the robot.(clockwise vel)

* **`robot_base_frame`** (string, default: "base")

    The name of the base frame of the robot.




# Final Result
## Stair climbing
![Stair climbing](https://github.com/guru-narayana/Aliengo_Nav-sim/blob/main/data/aliengo_gait.gif)
