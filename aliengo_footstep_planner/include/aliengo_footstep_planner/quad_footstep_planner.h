#ifndef __QUADFOOTPLANNER_H__
#define __QUADFOOTPLANNER_H__

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include "aliengo_msgs/transition_foothold.h"

using namespace std;
using namespace grid_map;

// user set params
extern double grid_map_resolution;
extern double max_steplength;
extern double min_steplength;
extern double resolution_steplength;
extern double favoured_steplength;

extern double prefered_stepcostFactor;
extern double obstacle_stepcostFactor;
extern double collision_costFactor;
extern double global_costFactor;
extern double edge_costFactor;

extern int horizon_length;
extern int steps_horizon;
extern bool visualize_plan;
extern vector<double> robot_config;
extern double collision_rect_length;
extern double collision_rect_width;
extern double collision_point_height;
extern double collision_threshold;
extern double collision_free_threshold;
extern double favoured_forward_vel;
extern double max_forward_vel;
extern double min_forward_vel;
extern double min_angular_vel;
extern double max_angular_vel;
extern double base_pose_yaw;
extern double base_pose_y;
extern double base_pose_x;
extern string robot_base_frame;
extern bool using_joystick;
extern int vx_samples;
extern int vtheta_samples;

// result data
extern vector<vector<double>> foot_steps;
extern bool collision_detected;
extern vector<double> planar_cost;


// Required data 
extern GridMap elev_map;
extern double joystick_vals[];
extern vector<vector<double>> current_robot_footsteps;
extern bool recived_global_plan;
extern nav_msgs::GetPlan get_plan;

// Plan functions 
void plan_footsteps(ros::Publisher poly_pub,ros::Publisher foot_marker_pub,ros::Publisher next_step_pub);
double collision_check(bool return_totalcost);
void update_planar_variance();
vector<vector<double>> foot_end_points();
vector<double> calc_omega_EP_angles(double step_length);

#endif
