#ifndef __QUADFOOTPLANNER_H__
#define __QUADFOOTPLANNER_H__

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace std;
using namespace grid_map;

// user set params
extern double grid_map_resolution;
extern double max_steplength;
extern double min_steplength;
extern double favoured_steplength;
extern int horizon_length;
extern vector<double> robot_config; 
extern double collision_rect_length;
extern double collision_rect_width;
extern double collision_point_height;
extern double collision_threshold;
extern double favoured_forward_vel;
extern double max_forward_vel;
extern double min_forward_vel;
extern double min_angular_vel;
extern double max_angular_vel;
extern double base_pose_yaw;


// result data
extern vector<vector<double>> foot_steps;
extern bool collision_detected;
extern vector<double> planar_cost;

// Required data 
extern GridMap elev_map;
extern double joystick_vals[];
extern vector<vector<double>> current_robot_footsteps;

// Plan functions 
void plan_footsteps();
void collision_check(ros::Publisher poly_pub);
void update_planar_variance();
void foot_end_points();
#endif
