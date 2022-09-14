#include "aliengo_footstep_planner/quad_footstep_planner.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "aliengo_msgs/quad_footstep.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

// foot_planning_node user set params
double grid_map_resolution;
double max_steplength;
double min_steplength;
double favoured_steplength;
int horizon_length;
vector<double> robot_config = {0,0,0,0,0};
double collision_rect_length;
double collision_rect_width;
double collision_point_height;
double collision_threshold;
double favoured_forward_vel;
double max_forward_vel;
double min_forward_vel;
double min_angular_vel;
double max_angular_vel;
double base_pose_yaw;


string grid_map_topic;
string joystick_topic;
bool using_joystick;


// global update params
GridMap elev_map;
double joystick_vals[] = {0.0,0.0};
vector<vector<double>> current_robot_footsteps;
bool map_available = false;


// ROS callback Functions
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
    joystick_vals[0] = msg->axes[0]; 
    joystick_vals[1] = msg->axes[1]; 
}
void gridmap_cb(const grid_map_msgs::GridMap& msg)
{
    GridMapRosConverter::fromMessage(msg, elev_map);
    map_available = true;
}

void current_foot_st_cb(const aliengo_msgs::quad_footstep& msg)
{
    current_robot_footsteps.clear();
    vector<double> temp0 = {msg.FL.x,msg.FL.y,msg.FL.z};
    current_robot_footsteps.push_back(temp0);
    vector<double> temp1 = {msg.FR.x,msg.FR.y,msg.FR.z};
    current_robot_footsteps.push_back(temp1);
    vector<double> temp2 = {msg.RL.x,msg.RL.y,msg.RL.z};
    current_robot_footsteps.push_back(temp2);
    vector<double> temp3 = {msg.RR.x,msg.RR.y,msg.RR.z};
    current_robot_footsteps.push_back(temp3);

}

void base_pose_cb(const geometry_msgs::PoseWithCovarianceStamped& msg){
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    base_pose_yaw = yaw;
}
// user set params load function
void get_params(ros::NodeHandle& nh){
    nh.param("Grid_map_topic", grid_map_topic, string("/elevation_mapping/elevation_map_raw"));
    nh.param("Grid_map_resolution", grid_map_resolution, 0.05);
    nh.param("max_steplength", max_steplength, 2*grid_map_resolution);
    nh.param("min_steplength", min_steplength, grid_map_resolution);
    nh.param("favoured_steplength", favoured_steplength, 2*grid_map_resolution);
    nh.param("using_joystick", using_joystick, true);
    nh.param("joystick_topic", joystick_topic, string("/joy"));
    nh.param("horizon_length", horizon_length, 13);
    nh.param("/robot_config/base_length",robot_config[0], 0.2399*2);
    nh.param("/robot_config/base_width",robot_config[1], 0.051*2);
    nh.param("/robot_config/L1",robot_config[2],  0.083);
    nh.param("/robot_config/L2",robot_config[3], 0.25);
    nh.param("/robot_config/L3",robot_config[4], 0.25);
    nh.param("/robot_config/collision_rect_length",collision_rect_length, 0.65);
    nh.param("/robot_config/collision_rect_width",collision_rect_width, 0.3);
    nh.param("/robot_config/collision_point_height",collision_point_height, 0.2);
    nh.param("/robot_config/collision_threshold",collision_threshold, 99.5);

    nh.param("/robot_config/max_forward_vel",max_forward_vel,  0.8);
    nh.param("/robot_config/min_forward_vel",min_forward_vel,  -0.5);   // m/s
    nh.param("/robot_config/min_angular_vel",min_angular_vel, -0.872); // rad/s
    nh.param("/robot_config/max_angular_vel",max_angular_vel,  0.872);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Quad_footstep_planner_node");
    ros::NodeHandle nh("Quad_footstep_planner");
    get_params(nh);

    ros::Subscriber joy_sub = nh.subscribe(joystick_topic, 1, joy_cb);
    ros::Subscriber local_gridmap_sub = nh.subscribe(grid_map_topic, 1, gridmap_cb);
    ros::Subscriber FL_force = nh.subscribe("/Aliengo_foot_crntstate", 1, current_foot_st_cb);
    ros::Subscriber base_pose_sub = nh.subscribe("/base_pose", 1, base_pose_cb);
    

    ros::Publisher poly_pub = nh.advertise<geometry_msgs::PolygonStamped>("/poly",1);

    ros::Rate loop_rate(1000);

    if(!using_joystick) joy_sub.shutdown();
    while (ros::ok()){
        ros::spinOnce();
        if(map_available){

    
        plan_footsteps();
        collision_check(poly_pub);
        
        loop_rate.sleep();

    }}
    
    return EXIT_SUCCESS;
}
