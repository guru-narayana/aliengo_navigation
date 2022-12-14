#include "aliengo_controller/controller.h"
#include "aliengo_controller/shifting_control.h"


void trot_a_step(ros::Publisher jnt_st_pub){

    vector<double> FL_init = current_robot_footsteps[0],FR_init = current_robot_footsteps[1], // change to local base
                    RL_init = current_robot_footsteps[2],RR_init = current_robot_footsteps[3],
                    FL0 = {robot_config[0]/2,robot_config[1]/2+robot_config[2],-robot_base_height} ,FR0 = {robot_config[0]/2,-robot_config[1]/2-robot_config[2],-robot_base_height},
                RL0 = {-robot_config[0]/2,robot_config[1]/2+robot_config[2],-robot_base_height} ,RR0 = {-robot_config[0]/2,-robot_config[1]/2-robot_config[2],-robot_base_height};
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    double R(sqrt(pow((robot_config[0])/2,2)+pow((robot_config[1] + 2*robot_config[2])/2,2))),
            theta(atan((robot_config[0])/(robot_config[1] + 2*robot_config[2])));
    double v = max(foot_holds.vel1,0.0),w = foot_holds.vel2,
           SL = max(favoured_steplength*(v/max_forward_vel),0.021),
            alpha = 2*asin(SL/(2*R)),
            vr = (SL*w)/alpha,
            freq = sqrt(9.81/robot_base_height);
    if(v==0 && w==0){
        shift_mode(jnt_st_pub);
        return;

    }
    if(swing){ // swing FL
        double thetaFR,thetaRL;
        vector<double> tempFR(2,0),tempRL(2,0);
        if(w<0){
        tempFR[0] = R*sin(theta+alpha);
        tempFR[1] = -R*cos(theta+alpha);
        tempRL[0] = -R*sin(theta+alpha);
        tempRL[1] = R*cos(theta+alpha);
        }
        else{
        tempFR[0] = R*sin(theta-alpha);
        tempFR[1] = -R*cos(theta-alpha);   
        tempRL[0] = -R*sin(theta-alpha);
        tempRL[1] = R*cos(theta-alpha);         
        }
        thetaFR = atan((tempFR[0]-FR0[0])/(tempFR[1]-FR0[1]));
        thetaRL = M_PI + atan(abs(tempRL[0]-RL0[0])/abs(tempRL[1]-RL0[1]));
        double thetaFR1 = atan2((v + vr*sin(thetaFR)),(vr*cos(thetaFR))),
                thetaRL1 = atan2((v + vr*sin(thetaRL)),(vr*cos(thetaRL)));
        vector<double> FR = {FR0[0] + SL*sin(thetaFR1)-FR_init[0],FR0[1] + SL*cos(thetaFR1)-FR_init[1]},
                        RL = {RL0[0] + SL*sin(thetaRL1)-RL_init[0],RL0[1] + SL*cos(thetaRL1)-RL_init[1]};

        double T = max(min(SL/sqrt( pow(v + vr*sin(thetaFR),2) + pow(vr*cos(thetaFR),2)),0.3),0.05);
        vector<vector<double>> A_fl = generate_swing_coefs(FL_init,FL0),
                                A_rr = generate_swing_coefs(RR_init,RR0);
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T)){
            double t = (ros::Time::now().toSec()-init_time),u = t/T;
            vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
            vector<vector<double>> FL_cnt_pos = Multiply(u_mat,A_fl),
                                    RR_cnt_pos = Multiply(u_mat,A_rr);
                     vector<double> FR_cnt_pos = {FR_init[0]-FR[0]*(sin(freq*t)/sin(freq*T)),FR_init[1]-FR[1]*(sin(freq*t)/sin(freq*T)),-robot_base_height},
                                    RL_cnt_pos = {RL_init[0]-RL[0]*(sin(freq*t)/sin(freq*T)),RL_init[1]-RL[1]*(sin(freq*t)/sin(freq*T)),-robot_base_height};
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
        double thetaFL,thetaRR;
        vector<double> tempRR(2,0),tempFL(2,0);
        if(w<0){
        tempFL[0] = R*sin(theta-alpha);
        tempFL[1] = R*cos(theta-alpha);
        tempRR[0] = -R*sin(theta-alpha);
        tempRR[1] = -R*cos(theta-alpha);
        }
        else{
        tempFL[0] = R*sin(theta+alpha);
        tempFL[1] = R*cos(theta+alpha);   
        tempRR[0] = -R*sin(theta+alpha);
        tempRR[1] = -R*cos(theta+alpha);         
        }
        thetaFL = atan((tempFL[0]-FL0[0])/(tempFL[1]-FL0[1]));
        thetaRR = M_PI - atan(abs(tempRR[0]-RR0[0])/abs(tempRR[1]-RR0[1]));

        double thetaRR1 = atan2((v + vr*sin(thetaRR)),(vr*cos(thetaRR))),
                thetaFL1 = atan2((v + vr*sin(thetaFL)),(vr*cos(thetaFL)));
        vector<double> FL = {FL0[0] + SL*sin(thetaFL1)-FL_init[0],FL0[1] + SL*cos(thetaFL1)-FL_init[1]},
                        RR = {RR0[0] + SL*sin(thetaRR1)-RR_init[0],RR0[1] + SL*cos(thetaRR1)-RR_init[1]};
        double T = max(min(SL/sqrt(pow(v + vr*sin(thetaFL),2)+pow(vr*cos(thetaFL),2)),0.3),0.05);
        vector<vector<double>> A_fr = generate_swing_coefs(FR_init,FR0),
                                A_rl = generate_swing_coefs(RL_init,RL0);
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T)){
            double t = (ros::Time::now().toSec()-init_time),u = t/T;
            vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
            vector<vector<double>> FR_cnt_pos = Multiply(u_mat,A_fr),
                                    RL_cnt_pos = Multiply(u_mat,A_rl);
                     vector<double> FL_cnt_pos = {FL_init[0]-FL[0]*(sin(freq*t)/sin(freq*T)),FL_init[1]-FL[1]*(sin(freq*t)/sin(freq*T)),-robot_base_height},
                                    RR_cnt_pos = {RR_init[0]-RR[0]*(sin(freq*t)/sin(freq*T)),RR_init[1]-RR[1]*(sin(freq*t)/sin(freq*T)),-robot_base_height};
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