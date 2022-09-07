#include "quad_footstep_planner.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


// foot_planning_node user set params
string grid_map_topic;
double grid_map_resolution;
double max_steplength;
double min_steplength;
bool using_joystick;
string joystick_topic;

// Global params
double joystick_vals[] = {0.0,0.0};


// ROS callback Functions
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
    joystick_vals[0] = msg->axes[0]; 
    joystick_vals[1] = msg->axes[1]; 
}
void gridmap_cb(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    cout<<"working1"<<endl;
}


// user set params load function
void get_params(ros::NodeHandle& nh){
    nh.param("Grid_map_topic", grid_map_topic, string("/elevation_mapping/elevation_map"));
    nh.param("Grid_map_resolution", grid_map_resolution, 5.0);
    nh.param("max_steplength", max_steplength, 3*grid_map_resolution);
    nh.param("min_steplength", min_steplength, grid_map_resolution);
    nh.param("using_joystick", using_joystick, true);
    nh.param("joystick_topic", joystick_topic, string("/joy"));

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "Quad_footstep_planner_node");
    ros::NodeHandle nh("Quad_footstep_planner");
    get_params(nh);

    ros::Subscriber joy_sub = nh.subscribe(joystick_topic, 1, joy_cb);
    ros::Subscriber local_gridmap_sub = nh.subscribe(grid_map_topic, 1, gridmap_cb);
    if(!using_joystick) joy_sub.shutdown();


    ros::spin();
    
    return EXIT_SUCCESS;
}
