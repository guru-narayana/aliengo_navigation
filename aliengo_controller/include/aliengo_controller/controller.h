#ifndef __ALIENGOCONTROL_H__
#define __ALIENGOCONTROL_H__

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <aliengo_kindym/aliengo_kindym.h>
#include <tf/tf.h>
#include "aliengo_msgs/quad_footstep.h"
#include "unitree_legged_msgs/Aliengo_Joint_controll.h"
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include "sensor_msgs/JointState.h"
#include "aliengo_msgs/transition_foothold.h"

using namespace std;

extern vector<double> robot_config;
extern string robot_base_frame;
extern double robot_verti_vel;
extern double robot_base_height;
extern double robot_swing_height;
extern unitree_legged_msgs::Aliengo_Joint_controll jnt_set_st;
extern vector<double> FL_current_xyz_st,RL_current_xyz_st,FR_current_xyz_st,RR_current_xyz_st;
extern int controller_rate;
extern bool swing;

extern double base_pose_yaw;
extern double base_pose_y;
extern double base_pose_z;
extern double base_pose_x;
extern vector<vector<double>> current_robot_footsteps;
extern aliengo_msgs::transition_foothold foot_holds;

extern void trot_a_step(ros::Publisher jnt_st_pub);
#endif