#include "aliengo_controller/controller.h"
#include "aliengo_controller/shifting_control.h"

void move_base(double m,double T,vector<double>&  FL_init,vector<double>&  FR_init,vector<double>&  RL_init,vector<double>&  RR_init,ros::Publisher jnt_st_pub
                                ,vector<double>&  FL1,vector<double>&  FR1,vector<double>&  RL1,vector<double>&  RR1,double x){
    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T)){
        double u = (ros::Time::now().toSec()-init_time)/T;
        vector<double> FL_cnt_pos = {FL_init[0]- x*u,FL_init[1]-m*u,FL_init[2]},
        RL_cnt_pos = {RL_init[0] - x*u,RL_init[1]-m*u,RL_init[2]},
        FR_cnt_pos = {FR_init[0] - x*u,FR_init[1]-m*u,FR_init[2]},
        RR_cnt_pos = {RR_init[0] - x*u,RR_init[1]-m*u,RR_init[2]};
        vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(FL_cnt_pos)),
                        RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(RR_cnt_pos)),
                        FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(FR_cnt_pos)),
                        RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(RL_cnt_pos));
        jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
        jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
        jnt_set_st.joint_positions[2] = -FR_req_jnt[2];
        jnt_set_st.joint_positions[9] = RL_req_jnt[0];
        jnt_set_st.joint_positions[10]= RL_req_jnt[1];
        jnt_set_st.joint_positions[11]= RL_req_jnt[2];
        jnt_set_st.joint_positions[3] = FL_req_jnt[0];
        jnt_set_st.joint_positions[4] = FL_req_jnt[1];
        jnt_set_st.joint_positions[5] = FL_req_jnt[2];
        jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
        jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
        jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
        jnt_st_pub.publish(jnt_set_st);
        frequency.sleep();
    }
    FL_init[1]-=m;FR_init[1]-=m;RR_init[1]-=m;RL_init[1]-=m;
    FL_init[0]-=x;FR_init[0]-=x;RR_init[0]-=x;RL_init[0]-=x;
    FL1[1]-=m;FR1[1]-=m;RR1[1]-=m;RL1[1]-=m;
    FL1[0]-=x;FR1[0]-=x;RR1[0]-=x;RL1[0]-=x;

}
void swng(ros::Publisher jnt_st_pub,vector<double>& iniat,vector<double> finl,int cls){
    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
        vector<vector<double>> A = generate_swing_coefs(iniat,finl);
    vector<double> cnt_pos,FL_req_jnt,FR_req_jnt,RL_req_jnt,RR_req_jnt;
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=swing_time)){
        double u = (ros::Time::now().toSec()-init_time)/swing_time;
        vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
        switch (cls)
        {
        case 0:
             cnt_pos = Multiply(u_mat,A)[0];
            FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(cnt_pos));
            jnt_set_st.joint_positions[3] = FL_req_jnt[0];
            jnt_set_st.joint_positions[4] = FL_req_jnt[1];
            jnt_set_st.joint_positions[5] = FL_req_jnt[2];
            break;
        case 1:
             cnt_pos = Multiply(u_mat,A)[0];
            FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(cnt_pos));
            jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
            jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
            jnt_set_st.joint_positions[2] = -FR_req_jnt[2];
            break;
        case 2:
             cnt_pos = Multiply(u_mat,A)[0];
            RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(cnt_pos));
            jnt_set_st.joint_positions[9] = RL_req_jnt[0];
            jnt_set_st.joint_positions[10]= RL_req_jnt[1];
            jnt_set_st.joint_positions[11]= RL_req_jnt[2];
            break;
        case 3:
            cnt_pos = Multiply(u_mat,A)[0];
            RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(cnt_pos));
            jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
            jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
            jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
            break;
        default:
            break;
        }
        jnt_st_pub.publish(jnt_set_st);
        frequency.sleep();
        ros::spinOnce();
        
    }
    iniat = cnt_pos;
}
void rotate_base(ros::Publisher jnt_st_pub,vector<double>&  FL_init,vector<double>&  FR_init,vector<double>&  RL_init,vector<double>&  RR_init,double omega){

}
void walk_a_step(ros::Publisher jnt_st_pub){
    vector<double> FL_init = current_robot_footsteps[0],FR_init = current_robot_footsteps[1], // change to local base
                    RL_init = current_robot_footsteps[2],RR_init = current_robot_footsteps[3];
    double X = foot_holds.base_pose.position.x,Y = foot_holds.base_pose.position.y,omega =foot_holds.base_pose.orientation.z ;
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    vector<double> FL1 = {foot_holds.FL1.x,foot_holds.FL1.y,foot_holds.FL1.z+toe_radius},FR1 = {foot_holds.FR1.x,foot_holds.FR1.y,foot_holds.FR1.z+toe_radius},
                RL1 = {foot_holds.RL1.x,foot_holds.RL1.y,foot_holds.RL1.z+toe_radius},RR1 = {foot_holds.RR1.x,foot_holds.RR1.y,foot_holds.RR1.z+toe_radius};
    move_base(walk_marigin,walk_marigin/walk_base_vel,FL_init,FR_init,RL_init,RR_init,jnt_st_pub,FL1,FR1,RL1,RR1,-walk_mariginx);
    swng(jnt_st_pub,FR_init,FR1,1);
    move_base(-2*walk_marigin,2*walk_marigin/walk_base_vel,FL_init,FR_init,RL_init,RR_init,jnt_st_pub,FL1,FR1,RL1,RR1,2*walk_mariginx);
    swng(jnt_st_pub,RL_init,RL1,2);
    move_base(0,2*walk_marigin/walk_base_vel,FL_init,FR_init,RL_init,RR_init,jnt_st_pub,FL1,FR1,RL1,RR1,-2*walk_mariginx);
    swng(jnt_st_pub,FL_init,FL1,0);
    move_base(2*walk_marigin,2*walk_marigin/walk_base_vel,FL_init,FR_init,RL_init,RR_init,jnt_st_pub,FL1,FR1,RL1,RR1,2*walk_mariginx);
    swng(jnt_st_pub,RR_init,RR1,3);
    move_base(-walk_marigin+Y,sqrt(pow(X-walk_mariginx,2)+pow(Y-walk_marigin,2))/walk_base_vel,FL_init,FR_init,RL_init,RR_init,jnt_st_pub,FL1,FR1,RL1,RR1,-walk_mariginx+X);
}

