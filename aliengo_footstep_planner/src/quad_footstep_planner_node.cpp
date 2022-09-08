#include "quad_footstep_planner.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


// foot_planning_node user set params
double grid_map_resolution;
double max_steplength;
double min_steplength;
double favoured_steplength;
int horizon_length;
vector<double> robot_config = {0,0,0,0,0}; 
double collision_rect_length;
double collision_rect_width;


string grid_map_topic;
string joystick_topic;
bool using_joystick;


// global update params
GridMap global_map;
double joystick_vals[] = {0.0,0.0};

// ROS callback Functions
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
    joystick_vals[0] = msg->axes[0]; 
    joystick_vals[1] = msg->axes[1]; 
}
void gridmap_cb(const grid_map_msgs::GridMap& msg)
{
    GridMapRosConverter::fromMessage(msg, global_map, {"elevation"});
}


// user set params load function
void get_params(ros::NodeHandle& nh){
    nh.param("Grid_map_topic", grid_map_topic, string("/elevation_mapping/elevation_map"));
    nh.param("Grid_map_resolution", grid_map_resolution, 5.0);
    nh.param("max_steplength", max_steplength, 3*grid_map_resolution);
    nh.param("min_steplength", min_steplength, grid_map_resolution);
    nh.param("favoured_steplength", favoured_steplength, 2*grid_map_resolution);
    nh.param("using_joystick", using_joystick, true);
    nh.param("joystick_topic", joystick_topic, string("/joy"));
    nh.param("horizon_length", horizon_length, 3);
    nh.param("/robot_config/base_length",robot_config[0], 0.2399*2);
    nh.param("/robot_config/base_width",robot_config[1], 0.051*2);
    nh.param("/robot_config/L1",robot_config[2],  0.083);
    nh.param("/robot_config/L2",robot_config[3], 0.25);
    nh.param("/robot_config/L3",robot_config[4], 0.25);
    nh.param("/robot_config/collision_rect_length",collision_rect_length, 0.54);
    nh.param("/robot_config/collision_rect_width",collision_rect_width, 0.15);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Quad_footstep_planner_node");
    ros::NodeHandle nh("Quad_footstep_planner");
    get_params(nh);

    ros::Subscriber joy_sub = nh.subscribe(joystick_topic, 1, joy_cb);
    ros::Subscriber local_gridmap_sub = nh.subscribe(grid_map_topic, 1, gridmap_cb);
    if(!using_joystick) joy_sub.shutdown();
    while (ros::ok()){
        ros::spinOnce();
        plan_footsteps();

    }
    
    return EXIT_SUCCESS;
}
