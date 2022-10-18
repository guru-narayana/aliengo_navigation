#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "aliengo_msgs/quad_footstep.h"
#include "geometry_msgs/WrenchStamped.h"
using namespace std;


aliengo_msgs::quad_footstep foot_status;

tf::StampedTransform FL_tf,FR_tf,RL_tf,RR_tf;

void FL_contact_cb(const geometry_msgs::WrenchStampedConstPtr& msg){
    if(msg->wrench.force.z > 10)foot_status.contact_state[0] = 1;
    else foot_status.contact_state[0] = 0;
}
void FR_contact_cb(const geometry_msgs::WrenchStampedConstPtr& msg){
    if(msg->wrench.force.z > 10) foot_status.contact_state[1] = 1;
    else foot_status.contact_state[1] = 0;
}
void RL_contact_cb(const geometry_msgs::WrenchStampedConstPtr& msg){
    if(msg->wrench.force.z > 10) foot_status.contact_state[2] = 1;
    else foot_status.contact_state[2] = 0;
}
void RR_contact_cb(const geometry_msgs::WrenchStampedConstPtr& msg){
    if(msg->wrench.force.z > 10) foot_status.contact_state[3] = 1;
    else foot_status.contact_state[3] = 0;
}
int main(int argc,char** argv){
    ros::init(argc,argv,"footstep_state_publisher");
    ros::NodeHandle nh;
    foot_status.contact_state = {true,true,true,true};
    
    ros::Subscriber FL_force = nh.subscribe("visual/FL_foot_contact/the_force", 1, FL_contact_cb);
    ros::Subscriber FR_force = nh.subscribe("visual/FR_foot_contact/the_force", 1, FR_contact_cb);
    ros::Subscriber RL_force = nh.subscribe("visual/RL_foot_contact/the_force", 1, RL_contact_cb);
    ros::Subscriber RR_force = nh.subscribe("visual/RR_foot_contact/the_force", 1, RR_contact_cb);

    ros::Publisher foot_pub = nh.advertise<aliengo_msgs::quad_footstep>("Aliengo_foot_crntstate", 1);
    tf::TransformListener listener;


    double foot_radius;
    nh.param("/robot_config/foot_radius",foot_radius,  0.0265);
    ros::Rate loop_rate(500);

    while(ros::ok()){        
        try{
            listener.lookupTransform("/base", "/FL_foot",ros::Time(0), FL_tf);
            listener.lookupTransform("/base", "/FR_foot",ros::Time(0), FR_tf);
            listener.lookupTransform("/base", "/RL_foot",ros::Time(0), RL_tf);
            listener.lookupTransform("/base", "/RR_foot",ros::Time(0), RR_tf);
            foot_status.FL.x = FL_tf.getOrigin().x();
            foot_status.FL.y = FL_tf.getOrigin().y();
            foot_status.FL.z = FL_tf.getOrigin().z();

            foot_status.FR.x = FR_tf.getOrigin().x();
            foot_status.FR.y = FR_tf.getOrigin().y();
            foot_status.FR.z = FR_tf.getOrigin().z();

            foot_status.RL.x = RL_tf.getOrigin().x();
            foot_status.RL.y = RL_tf.getOrigin().y();
            foot_status.RL.z = RL_tf.getOrigin().z();

            foot_status.RR.x = RR_tf.getOrigin().x();
            foot_status.RR.y = RR_tf.getOrigin().y();
            foot_status.RR.z = RR_tf.getOrigin().z();
            

            foot_pub.publish(foot_status);

    
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
        loop_rate.sleep();
        ros::spinOnce();

    }
    return 0;

}