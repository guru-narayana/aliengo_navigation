
#include "aliengo_controller/controller.h"
#include "aliengo_controller/shifting_control.h"

// Global params
vector<double> robot_config = {0,0,0,0,0};
string robot_base_frame;
double robot_verti_vel;
double robot_base_height;
double robot_swing_height;
unitree_legged_msgs::Aliengo_Joint_controll jnt_set_st;


//Regular updated params
double base_pose_yaw;
double base_pose_y;
double base_pose_z;
double base_pose_x;
vector<vector<double>> current_robot_footsteps;
vector<double> FL_current_xyz_st = {0,0,0}; // actual  Front Left foot position wrt thighs coordinate frame
vector<double> RL_current_xyz_st = {0,0,0};
vector<double> FR_current_xyz_st = {0,0,0};
vector<double> RR_current_xyz_st = {0,0,0};

//conditional params
bool nrecvd_callback1 = true,nrecvd_callback2 = true;





// Subsricptions call backs
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
}

void current_foot_st_cb(const aliengo_msgs::quad_footstep& msg){
    nrecvd_callback2 = false;
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
void joint_current_act_state_callback(const sensor_msgs::JointState::ConstPtr& msg){
    nrecvd_callback1 = false;
    vector<double> FL_current_jnt_st = {0,0,0};
    vector<double> RL_current_jnt_st = {0,0,0}; 
    vector<double> FR_current_jnt_st = {0,0,0};
    vector<double> RR_current_jnt_st = {0,0,0}; 

    FL_current_jnt_st[0] = (double)msg->position[1];
    FL_current_jnt_st[1] = (double)msg->position[2];
    FL_current_jnt_st[2] = (double)msg->position[0];
    
    FR_current_jnt_st[0] = (double)msg->position[1+3];
    FR_current_jnt_st[1] = -(double)msg->position[2+3];
    FR_current_jnt_st[2] = -(double)msg->position[0+3];

    RL_current_jnt_st[0] = (double)msg->position[1+6];
    RL_current_jnt_st[1] = (double)msg->position[2+6];
    RL_current_jnt_st[2] = (double)msg->position[0+6];

    RR_current_jnt_st[0] = (double)msg->position[1+9];
    RR_current_jnt_st[1] = -(double)msg->position[2+9];
    RR_current_jnt_st[2] = -(double)msg->position[0+9];

    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    FL_current_xyz_st = quad_kinem_g.Left_Leg_FK(FL_current_jnt_st);
    RL_current_xyz_st = quad_kinem_g.Left_Leg_FK(RL_current_jnt_st);
    FR_current_xyz_st = quad_kinem_g.Right_Leg_FK(FR_current_jnt_st);
    RR_current_xyz_st = quad_kinem_g.Right_Leg_FK(RR_current_jnt_st);
   // cout<<FR_current_xyz_st[0]<<"  " <<FR_current_xyz_st[1]<<"   "<<FR_current_xyz_st[2]<<"\n";
}

void get_params(ros::NodeHandle& nh){
    nh.param("/robot_config/base_length",robot_config[0], 0.2399*2);
    nh.param("/robot_config/base_width",robot_config[1], 0.051*2);
    nh.param("/robot_config/L1",robot_config[2],  0.083);
    nh.param("/robot_config/L2",robot_config[3], 0.25);
    nh.param("/robot_config/L3",robot_config[4], 0.25);
    nh.param("/robot_config/robot_base_frame",robot_base_frame,  string("/base"));

    nh.param("robot_verti_vel",robot_verti_vel, 0.06);
    nh.param("robot_base_height",robot_base_height, 0.35);
    nh.param("robot_swing_height",robot_swing_height, 0.5);

}

int main(int argc,char** argv){
    ros::init(argc,argv,"Quad_controller_node");
    ros::NodeHandle nh("Quad_controller");

    ros::Subscriber base_pose_sub = nh.subscribe("/base_pose", 1, base_pose_cb);
    ros::Subscriber  crnt_foot_st_sub = nh.subscribe("/Aliengo_foot_crntstate", 1, current_foot_st_cb);
    ros::Subscriber crnt_jnt_st_sub = nh.subscribe("/aliengo_gazebo/joint_states", 1, joint_current_act_state_callback); 
    
    ros::Publisher jnt_st_pub = nh.advertise<unitree_legged_msgs::Aliengo_Joint_controll>("/Aliengo_jnt_req_state", 1);
    get_params(nh);
    while(nrecvd_callback1 || nrecvd_callback2){
        ros::spinOnce();
    }
    height_adjust(jnt_st_pub);
    ros::Duration(0.5).sleep();
    shift_mode(jnt_st_pub);
    ros::spinOnce();

}