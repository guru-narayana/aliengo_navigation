#include "ros/ros.h"

#include "aliengo_kindym/aliengo_kindym.h"
#include "aliengo_msgs/quad_footstep.h"

using namespace std;
int main(int argc,char** argv){
    ros::init(argc,argv,"footstep_state_publisher");
    ros::NodeHandle nh;

    vector<double> robot_config = {0,0,0,0,0};
    nh.param("/robot_config/base_length",robot_config[0], 0.2399*2);
    nh.param("/robot_config/base_width",robot_config[1], 0.051*2);
    nh.param("/robot_config/L1",robot_config[2],  0.083);
    nh.param("/robot_config/L2",robot_config[3], 0.25);
    nh.param("/robot_config/L3",robot_config[4], 0.25);
    quad_kinem quad(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    
    
    
    return 0;

}