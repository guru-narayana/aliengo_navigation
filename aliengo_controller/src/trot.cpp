#include "aliengo_controller/controller.h"
#include "aliengo_controller/shifting_control.h"


void trot_a_step(ros::Publisher jnt_st_pub){

    vector<double> FL_init = current_robot_footsteps[0],FR_init = current_robot_footsteps[1], // change to local base
                    RL_init = current_robot_footsteps[2],RR_init = current_robot_footsteps[3];
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    vector<double> FL1 = {foot_holds.FL1.x,foot_holds.FL1.y,foot_holds.FL1.z},FR1 = {foot_holds.FR1.x,foot_holds.FR1.y,foot_holds.FR1.z},
                RL1 = {foot_holds.RL1.x,foot_holds.RL1.y,foot_holds.RL1.z},RR1 = {foot_holds.RR1.x,foot_holds.RR1.y,foot_holds.RR1.z},
                FL2 = {foot_holds.FL2.x,foot_holds.FL2.y,foot_holds.FL2.z},FR2 = {foot_holds.FR2.x,foot_holds.FR2.y,foot_holds.FR2.z},
                RL2 = {foot_holds.RL2.x,foot_holds.RL2.y,foot_holds.RL2.z},RR2 = {foot_holds.RR2.x,foot_holds.RR2.y,foot_holds.RR2.z},
                FL0 = {robot_config[0]/2,robot_config[1]/2+robot_config[2]} ,FR0 = {robot_config[0]/2,-robot_config[1]/2-robot_config[2]},
                RL0 = {-robot_config[0]/2,robot_config[1]/2+robot_config[2]} ,RR0 = {-robot_config[0]/2,-robot_config[1]/2-robot_config[2]};

    double vel1 = foot_holds.vel1,vel2 = foot_holds.vel2,w = sqrt(9.81/base_pose_z);
    if(swing){ // swing FL
        double SL = sqrt(pow(RL_init[0]-RL0[0],2)+pow(RL_init[1]-RL0[1],2)),T = min(max(SL/vel1,0.2),0.4);
        cout<<T<<endl;
        vector<double> com_ep = {(FR_init[0]+RL_init[0])/2,(FR_init[1]+RL_init[1])/2}; 
        double com_theta = - atan(abs(FR_init[0]/FR_init[1])) +  atan(abs(FR0[0]/FR0[1]));
        vector<double> FL_travld = {FL0[0]*cos(com_theta) - FL0[1]*sin(com_theta) - com_ep[0] - FL0[0], FL0[0]*sin(com_theta) + FL0[1]*cos(com_theta) - com_ep[1] - FL0[1]},
                        RR_travld = {RR0[0]*cos(com_theta) - RR0[1]*sin(com_theta) - com_ep[0] - RR0[0], RR0[0]*sin(com_theta) + RR0[1]*cos(com_theta) - com_ep[1] - RR0[1]};
        vector<double> FL_final = FL1,RR_final = RR1;
        FL_final = {FL2[0]+FL_travld[0],FL2[1]+FL_travld[1],-robot_base_height}; 
        RR_final = {RR2[0]+RR_travld[0],RR2[1]+RR_travld[1],-robot_base_height};
        cout<<FL_travld[0]<<"  "<<FL_travld[1]<<endl;
        vector<vector<double>> A_fl = generate_swing_coefs(FL_init,FL_final),A_rr = generate_swing_coefs(RR_init,RR_final);
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T)){
            double t = (ros::Time::now().toSec()-init_time),u = t/T;
            vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
            vector<vector<double>> FL_cnt_pos = Multiply(u_mat,A_fl),
                                    RR_cnt_pos = Multiply(u_mat,A_rr);
                     vector<double> FR_cnt_pos = {FR_init[0]-(FR_init[0]-FR0[0])*(sin(w*t)/sin(w*T)),FR_init[1]-(FR_init[1]-FR0[1])*(sin(w*t)/sin(w*T)),-robot_base_height},
                                    RL_cnt_pos = {RL_init[0]-(RL_init[0]-RL0[0])*(sin(w*t)/sin(w*T)),RL_init[1]-(RL_init[1]-RL0[1])*(sin(w*t)/sin(w*T)),-robot_base_height};
                    vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(FL_cnt_pos[0])),
                            RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(RR_cnt_pos[0])),
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
    
    } 
    else{
        double SL = sqrt(pow(FL_init[0]-FL0[0],2)+pow(FL_init[1]-FL0[1],2)),T = min(max(SL/vel1,0.2),0.4);
        cout<<T<<endl;
        vector<double> com_ep = {(FL_init[0]+RR_init[0])/2,(FL_init[1]+RR_init[1])/2}; 
        double com_theta = - atan(abs(FL_init[0]/FL_init[1])) +  atan(abs(FL0[0]/FL0[1]));
        vector<double> FR_travld = {FR0[0]*cos(com_theta) - FR0[1]*sin(com_theta) - com_ep[0] - FR0[0], FR0[0]*sin(com_theta) + FR0[1]*cos(com_theta) - com_ep[1] - FR0[1]},
                        RL_travld = {RL0[0]*cos(com_theta) - RL0[1]*sin(com_theta) - com_ep[0] - RL0[0], RL0[0]*sin(com_theta) + RL0[1]*cos(com_theta) - com_ep[1] - RL0[1]};
        vector<double> FR_final = FR1,RL_final = RL1;
        FR_final = {FR2[0]+FR_travld[0],FR2[1]+FR_travld[1],-robot_base_height}; 
        RL_final = {RL2[0]+RL_travld[0],RL2[1]+RL_travld[1],-robot_base_height};
        vector<vector<double>> A_fr = generate_swing_coefs(FR_init,FR_final),A_rl = generate_swing_coefs(RL_init,RL_final);
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T)){
            double t = (ros::Time::now().toSec()-init_time),u = t/T;
            vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
            vector<vector<double>> FR_cnt_pos = Multiply(u_mat,A_fr),
                                    RL_cnt_pos = Multiply(u_mat,A_rl);
                     vector<double> FL_cnt_pos = {FL_init[0]-(FL_init[0]-FL0[0])*(sin(w*t)/sin(w*T)),FL_init[1]-(FL_init[1]-FL0[1])*(sin(w*t)/sin(w*T)),-robot_base_height},
                                    RR_cnt_pos = {RR_init[0]-(RR_init[0]-RR0[0])*(sin(w*t)/sin(w*T)),RR_init[1]-(RR_init[1]-RR0[1])*(sin(w*t)/sin(w*T)),-robot_base_height};
                    vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(FL_cnt_pos)),
                            RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(RR_cnt_pos)),
                            FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(FR_cnt_pos[0])),
                            RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(RL_cnt_pos[0]));

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
    }
    swing = !swing;

}