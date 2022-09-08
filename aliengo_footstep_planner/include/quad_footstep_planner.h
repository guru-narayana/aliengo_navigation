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

// result data
extern vector<vector<double>> foot_steps;
extern bool collision_detected;
extern vector<double> planar_cost;

// Required data 
extern GridMap global_map;
extern double joystick_vals[];
extern vector<vector<double>> current_robot_footsteps;

// Plan functions 
void plan_footsteps();
void check_collision();
void update_planar_cost();

#endif
