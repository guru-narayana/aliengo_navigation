#include "aliengo_controller/controller.h"
#include "aliengo_controller/shifting_control.h"


void trot_a_step(ros::Publisher jnt_st_pub){

    vector<double> FL_init = current_robot_footsteps[0],FR_init = current_robot_footsteps[1], // change to local base
                    RL_init = current_robot_footsteps[2],RR_init = current_robot_footsteps[3]; 
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    vector<double> FL1 = {foot_holds.FL1.x,foot_holds.FL1.y,foot_holds.FL1.z},FR1 = {foot_holds.FR1.x,foot_holds.FR1.y,foot_holds.FR1.z},
                RL1 = {foot_holds.RL1.x,foot_holds.RL1.y,foot_holds.RL1.z},RR1 = {foot_holds.RR1.x,foot_holds.RR1.y,foot_holds.RR1.z},
                FL2 = {foot_holds.FL2.x,foot_holds.FL2.y,foot_holds.FL2.z},FR2 = {foot_holds.FR2.x,foot_holds.FR2.y,foot_holds.FR2.z},
                RL2 = {foot_holds.RL2.x,foot_holds.RL2.y,foot_holds.RL2.z},RR2 = {foot_holds.RR2.x,foot_holds.RR2.y,foot_holds.RR2.z};
    double vel1 = foot_holds.vel1,vel2 = foot_holds.vel2,w = sqrt(9.81/base_pose_z);

    if(swing){ // swing FL

        vector<double> com_futr_pos = {(FR_init[0]+RL_init[0])/2,(FR_init[1]+RL_init[1])/2}; 
        double SL = sqrt(pow(com_futr_pos[0],2)+pow(com_futr_pos[0],2)),T = max(SL/vel1,0.25);
        cout<<SL<<"  "<<T<<endl;
        vector<vector<double>> A_fl,A_rr;
        if(SL<0.25*(sqrt(pow(FR1[0]-(robot_config[0]/2),2)+pow(FR1[1]+(robot_config[1]/2 + robot_config[2]),2)))){
            vector<double> FL_final = {FL1[0] - com_futr_pos[0],FL1[1] - com_futr_pos[1],FL1[2]},RR_final = {RR1[0] - com_futr_pos[0],RR1[1] - com_futr_pos[1],RR1[2]};
            A_fl = generate_swing_coefs(FL_init,FL_final);A_rr = generate_swing_coefs(RR_init,RR_final);
        }else{
            vector<double> FL_final = {FL2[0] - com_futr_pos[0],FL2[1] - com_futr_pos[1],FL2[2]},RR_final = {RR2[0] - com_futr_pos[0],RR2[1] - com_futr_pos[1],RR2[2]};
            A_fl = generate_swing_coefs(FL_init,FL_final);A_rr = generate_swing_coefs(RR_init,RR_final);
        }
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        while(ros::Time::now().toSec()-init_time<=T){
            double t = (ros::Time::now().toSec()-init_time),u = t/T;
            vector<double> LFR = {FR_init[0]-(robot_config[0]/2),FR_init[1]+(robot_config[1]/2 + robot_config[2])},
                            LRL = {RL_init[0]+(robot_config[0]/2),RL_init[1]-(robot_config[1]/2 + robot_config[2])};
            vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
            vector<vector<double>> FL_cnt_pos = Multiply(u_mat,A_fl),
                                    RR_cnt_pos = Multiply(u_mat,A_rr);
                     vector<double> FR_cnt_pos = {FR_init[0]-LFR[0]*(sin(w*t)/sin(w*T)),FR_init[1]-LFR[1]*(sin(w*t)/sin(w*T)),FR_init[2]},
                                    RL_cnt_pos = {RL_init[0]-LRL[0]*(sin(w*t)/sin(w*T)),RL_init[1]-LRL[1]*(sin(w*t)/sin(w*T)),RL_init[2]};
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




    else{ // swing FR
        vector<double> com_futr_pos = {(FR_init[0]+RL_init[0])/2,(FR_init[1]+RL_init[1])/2}; 
        double SL = sqrt(pow(com_futr_pos[0],2)+pow(com_futr_pos[0],2)),T = max(SL/vel1,0.25);
        cout<<SL<<"  "<<T<<endl;
        vector<vector<double>> A_fr,A_rl;
        if(SL<0.25*(sqrt(pow(FL1[0]-(robot_config[0]/2),2)+pow(FL1[1]-(robot_config[1]/2 + robot_config[2]),2)))){
            vector<double> FR_final = {FR1[0] - com_futr_pos[0],FR1[1] - com_futr_pos[1],FL1[2]},RL_final = {RL1[0] - com_futr_pos[0],RL1[1] - com_futr_pos[1],RL1[2]};
            A_fr = generate_swing_coefs(FR_init,FR_final);A_rl = generate_swing_coefs(RL_init,RL_final);
        }else{
            vector<double> FR_final = {FR2[0] - com_futr_pos[0],FR2[1] - com_futr_pos[1],FR2[2]},RL_final = {RL2[0] - com_futr_pos[0],RL2[1] - com_futr_pos[1],RL2[2]};
            A_fr = generate_swing_coefs(FR_init,FR_final);A_rl = generate_swing_coefs(RL_init,RL_final);
        }
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        while(ros::Time::now().toSec()-init_time<=T){
            double t = (ros::Time::now().toSec()-init_time),u = t/T;
            vector<double> LFL = {FL_init[0]- (robot_config[0]/2),FL_init[1]-(robot_config[1]/2 + robot_config[2])},
                            LRR = {RR_init[0]+(robot_config[0]/2),RR_init[1]+(robot_config[1]/2 + robot_config[2])};
            vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
            vector<vector<double>> FR_cnt_pos = Multiply(u_mat,A_fr),
                                    RL_cnt_pos = Multiply(u_mat,A_rl);
                     vector<double> FL_cnt_pos = {FL_init[0]-LFL[0]*(sin(w*t)/sin(w*T)),FL_init[1]-LFL[1]*(sin(w*t)/sin(w*T)),FL_init[2]},
                                    RR_cnt_pos = {RR_init[0]-LRR[0]*(sin(w*t)/sin(w*T)),RR_init[1]-LRR[1]*(sin(w*t)/sin(w*T)),RR_init[2]};
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
    if(swing) cout<<"true"<<endl;
    else cout<<"false"<<endl;
    swing  = !swing;

}