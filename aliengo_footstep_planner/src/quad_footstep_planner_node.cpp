#include "aliengo_footstep_planner/quad_footstep_planner.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "aliengo_msgs/quad_footstep.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

// foot_planning_node user set params
double grid_map_resolution;
double max_steplength;
double min_steplength;
double favoured_steplength;
double resolution_steplength;
double prefered_stepcostFactor;
double obstacle_stepcostFactor;
double collision_costFactor;
double edge_costFactor;
double global_costFactor;
bool visualize_plan;
int horizon_length;
int steps_horizon;
int vtheta_samples;
int vx_samples;
vector<double> robot_config = {0,0,0,0,0};
int contacts = 0; 

double collision_rect_length;
double collision_rect_width;
double collision_point_height;
double collision_threshold;
double collision_free_threshold;
double favoured_forward_vel;
double max_forward_vel;
double min_forward_vel;
double min_angular_vel;
double max_angular_vel;
double base_pose_yaw;
double base_pose_y;
double base_pose_x;
double base_pose_z;
string robot_base_frame;
string goal_topic;


string grid_map_topic;
string joystick_topic;
bool using_joystick;


// global update params
GridMap elev_map;
double joystick_vals[] = {0.0,0.0};
vector<vector<double>> current_robot_footsteps;
bool map_available = false;
ros::ServiceClient client;
geometry_msgs::PoseStamped start;
nav_msgs::GetPlan get_plan;
bool recived_global_plan = false;

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
    contacts = msg.contact_state[0]+ msg.contact_state[1]+
                msg.contact_state[2]+ msg.contact_state[3];
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
    base_pose_x = msg.pose.pose.position.x;
    base_pose_y = msg.pose.pose.position.y;
    base_pose_z = msg.pose.pose.position.z;
    start.header = msg.header;
    start.pose = msg.pose.pose;
}

void goal_pose_cb(const geometry_msgs::PoseStamped& goal){
    recived_global_plan = true;
    get_plan.request.start = start;
    get_plan.request.goal = goal;
    

    if (!client.call(get_plan))
    {
        ROS_ERROR("Failed to call service make_plan");
    }
}
// user set params load function
void get_params(ros::NodeHandle& nh){
    nh.param("Grid_map_topic", grid_map_topic, string("/elevation_mapping/elevation_map_raw"));
    nh.param("Grid_map_resolution", grid_map_resolution, 0.03);
    nh.param("visualize_plan", visualize_plan, true);
    nh.param("max_steplength", max_steplength, 0.05);
    nh.param("min_steplength", min_steplength, 0.02);
    nh.param("favoured_steplength", favoured_steplength, 0.03);
    nh.param("resolution_steplength", resolution_steplength, 0.01);
    nh.param("prefered_stepcostFactor", prefered_stepcostFactor, 0.4);
    nh.param("collision_costFactor", collision_costFactor, 1.2);
    nh.param("global_costFactor", global_costFactor, 0.4);
    nh.param("obstacle_stepcostFactor", obstacle_stepcostFactor, 0.6);
    nh.param("edge_costFactor", edge_costFactor, 1.0);
    nh.param("using_joystick", using_joystick, false);
    nh.param("joystick_topic", joystick_topic, string("/joy"));
    nh.param("horizon_length", horizon_length, 13);
    nh.param("foot_plan_horizon", steps_horizon, 5);
    nh.param("vx_samples", vx_samples, 5);
    nh.param("vtheta_samples", vtheta_samples, 20);
    nh.param("global_goal_topic",goal_topic,  string("/aliengo_goal"));
    nh.param("/robot_config/base_length",robot_config[0], 0.2399*2);
    nh.param("/robot_config/base_width",robot_config[1], 0.051*2);
    nh.param("/robot_config/L1",robot_config[2],  0.083);
    nh.param("/robot_config/L2",robot_config[3], 0.25);
    nh.param("/robot_config/L3",robot_config[4], 0.25);
    nh.param("/robot_config/collision_rect_length",collision_rect_length, 0.65);
    nh.param("/robot_config/collision_rect_width",collision_rect_width, 0.3);
    nh.param("/robot_config/collision_point_height",collision_point_height, 0.2);
    nh.param("/robot_config/collision_threshold",collision_threshold, 0.0);
    nh.param("/robot_config/collision_free_threshold",collision_free_threshold, 100.0);
    nh.param("/robot_config/max_forward_vel",max_forward_vel,  0.2);
    nh.param("/robot_config/min_forward_vel",min_forward_vel,  -0.5);   // m/s
    nh.param("/robot_config/min_angular_vel",min_angular_vel, -0.872); // rad/s
    nh.param("/robot_config/max_angular_vel",max_angular_vel,  0.872);
    nh.param("/robot_config/robot_base_frame",robot_base_frame,  string("/base"));
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
    ros::Subscriber goal_pose_sub = nh.subscribe(goal_topic, 1, goal_pose_cb);


    client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    

    ros::Publisher poly_pub = nh.advertise<geometry_msgs::PolygonStamped>("/poly",1);
    ros::Publisher foot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("foot_visualization_marker", 1000);
    ros::Publisher next_foot_pub = nh.advertise<aliengo_msgs::transition_foothold >("transition_step_info", 1000);

    ros::Rate loop_rate(1000);

    if(!using_joystick) joy_sub.shutdown();
    while (ros::ok()){
        ros::spinOnce();
        if(map_available && (contacts >= 2)){
        plan_footsteps(poly_pub,foot_marker_pub,next_foot_pub);
        loop_rate.sleep();

    }}
    
    return EXIT_SUCCESS;
}
