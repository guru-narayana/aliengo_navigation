#include "aliengo_controller/controller.h"
#include "aliengo_controller/shifting_control.h"


double get_cubic_point(double x1, double x2,double u){
    return x1 -(10*(x1 - x2))*pow(u,3) + (15*(x1 - x2))*pow(u,4) -(6*(x1 - x2))*pow(u,5);
}

vector<double> traingulate(tf::StampedTransform& P1,tf::StampedTransform& P2,tf::StampedTransform& P3,geometry_msgs::PointStamped& p4,tf::StampedTransform& Pc){
    double m41 = (P1.getOrigin().y()-p4.point.y)/(P1.getOrigin().x()-p4.point.x),m23 = (P2.getOrigin().y()-P3.getOrigin().y())/(P2.getOrigin().x()-P3.getOrigin().x());
    double c41 = ((P1.getOrigin().x()*p4.point.y)-(p4.point.x*P1.getOrigin().y()))/(P1.getOrigin().x()-p4.point.x),
            c23 = ((P3.getOrigin().y()*P2.getOrigin().x())-(P2.getOrigin().y()*P3.getOrigin().x()))/(P2.getOrigin().x()-P3.getOrigin().x());
    double pkx = (c41-c23)/(m23-m41), pky = (m23*c41-c23*m41)/(m23-m41);
    double pcx = (P1.getOrigin().x() + P3.getOrigin().x() + pkx)/3,pcy = (P1.getOrigin().y() + P3.getOrigin().y() + pky)/3;
    double ratio_coeff = 0.7;
    double pcx1 = pcx*ratio_coeff + pkx*(1-ratio_coeff),pcy1 = pcy*ratio_coeff + pky*(1-ratio_coeff);

    vector<double> delta_pc  = {pcx1-Pc.getOrigin().x(),pcy1-Pc.getOrigin().y()};
    return delta_pc;
}

void move_base1(tf::TransformListener& listener,geometry_msgs::PointStamped&  RL1,ros::Publisher jnt_st_pub){
    while(ros::ok()){
    try{
    ros::spinOnce();
    tf::StampedTransform FR_tf,FL_tf,RR_tf,RL_tf,base_tf;
    listener.lookupTransform("/world", "/FL_foot",ros::Time(0), FL_tf);
    listener.lookupTransform("/world", "/FR_foot",ros::Time(0), FR_tf);
    listener.lookupTransform("/world", "/RL_foot",ros::Time(0), RL_tf);
    listener.lookupTransform("/world", "/RR_foot",ros::Time(0), RR_tf);
    listener.lookupTransform("/world", "/base",ros::Time(0), base_tf);
    vector<double> pc_delta = traingulate(FR_tf,FL_tf,RR_tf,RL1,base_tf);


    geometry_msgs::PointStamped FL1,FR1,RL1,RR1;
    geometry_msgs::PointStamped FL1b,FR1b,RL1b,RR1b;

    FL1.header.frame_id = "/world";
    FL1.header.stamp = ros::Time();
    FL1.point.x = FL_tf.getOrigin().x()-pc_delta[0];
    FL1.point.y = FL_tf.getOrigin().y()-pc_delta[1];
    FL1.point.z = FL_tf.getOrigin().z();

    FR1.header.frame_id = "/world";
    FR1.header.stamp = ros::Time();
    FR1.point.x = FR_tf.getOrigin().x()-pc_delta[0];
    FR1.point.y = FR_tf.getOrigin().y()-pc_delta[1];
    FR1.point.z = FR_tf.getOrigin().z();

    RL1.header.frame_id = "/world";
    RL1.header.stamp = ros::Time();
    RL1.point.x = RL_tf.getOrigin().x()-pc_delta[0];
    RL1.point.y = RL_tf.getOrigin().y()-pc_delta[1];
    RL1.point.z = RL_tf.getOrigin().z();

    RR1.header.frame_id = "/world";
    RR1.header.stamp = ros::Time();
    RR1.point.x = RR_tf.getOrigin().x()-pc_delta[0];
    RR1.point.y = RR_tf.getOrigin().y()-pc_delta[1];
    RR1.point.z = RR_tf.getOrigin().z();

    listener.transformPoint("/base", FL1, FL1b);
    listener.transformPoint("/base", FR1, FR1b);
    listener.transformPoint("/base", RL1, RL1b);
    listener.transformPoint("/base", RR1, RR1b);

    tf::StampedTransform FRb_tf,FLb_tf,RRb_tf,RLb_tf;
    listener.lookupTransform("/base", "/FL_foot",ros::Time(0), FLb_tf);
    listener.lookupTransform("/base", "/FR_foot",ros::Time(0), FRb_tf);
    listener.lookupTransform("/base", "/RL_foot",ros::Time(0), RLb_tf);
    listener.lookupTransform("/base", "/RR_foot",ros::Time(0), RRb_tf);


    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=walk_base_time)){
        double u = (ros::Time::now().toSec()-init_time)/(walk_base_time);
        vector<double> FL_cnt_pos = {get_cubic_point(FLb_tf.getOrigin().x(),FL1b.point.x,u),
                                        get_cubic_point(FLb_tf.getOrigin().y(),FL1b.point.y,u),
                                        get_cubic_point(FLb_tf.getOrigin().z(),FL1b.point.z,u)},

                        RL_cnt_pos = {get_cubic_point(RLb_tf.getOrigin().x(),RL1b.point.x,u),
                                        get_cubic_point(RLb_tf.getOrigin().y(),RL1b.point.y,u),
                                            get_cubic_point(RLb_tf.getOrigin().z(),RL1b.point.z,u)},

                        FR_cnt_pos = {get_cubic_point(FRb_tf.getOrigin().x(),FR1b.point.x,u),
                                        get_cubic_point(FRb_tf.getOrigin().y(),FR1b.point.y,u),
                                            get_cubic_point(FRb_tf.getOrigin().z(),FR1b.point.z,u)},

                        RR_cnt_pos = {get_cubic_point(RRb_tf.getOrigin().x(),RR1b.point.x,u),
                                        get_cubic_point(RRb_tf.getOrigin().y(),RR1b.point.y,u),
                                            get_cubic_point(RRb_tf.getOrigin().z(),RR1b.point.z,u)};

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
        ros::spinOnce();
        frequency.sleep();
    }

    break;
    }
    catch (tf::TransformException ex){
    //ROS_ERROR("%s",ex.what());
    }
    }


}

void move_base2(tf::TransformListener& listener,geometry_msgs::PointStamped&  RR1,ros::Publisher jnt_st_pub){
    while(ros::ok()){
    try{
    ros::spinOnce();
    tf::StampedTransform FR_tf,FL_tf,RR_tf,RL_tf,base_tf;
    listener.lookupTransform("/world", "/FL_foot",ros::Time(0), FL_tf);
    listener.lookupTransform("/world", "/FR_foot",ros::Time(0), FR_tf);
    listener.lookupTransform("/world", "/RL_foot",ros::Time(0), RL_tf);
    listener.lookupTransform("/world", "/RR_foot",ros::Time(0), RR_tf);
    listener.lookupTransform("/world", "/base",ros::Time(0), base_tf);
    vector<double> pc_delta = traingulate(FL_tf,FR_tf,RL_tf,RR1,base_tf);


    geometry_msgs::PointStamped FL1,FR1,RL1,RR1;
    geometry_msgs::PointStamped FL1b,FR1b,RL1b,RR1b;

    FL1.header.frame_id = "/world";
    FL1.header.stamp = ros::Time();
    FL1.point.x = FL_tf.getOrigin().x()-pc_delta[0];
    FL1.point.y = FL_tf.getOrigin().y()-pc_delta[1];
    FL1.point.z = FL_tf.getOrigin().z();

    FR1.header.frame_id = "/world";
    FR1.header.stamp = ros::Time();
    FR1.point.x = FR_tf.getOrigin().x()-pc_delta[0];
    FR1.point.y = FR_tf.getOrigin().y()-pc_delta[1];
    FR1.point.z = FR_tf.getOrigin().z();

    RL1.header.frame_id = "/world";
    RL1.header.stamp = ros::Time();
    RL1.point.x = RL_tf.getOrigin().x()-pc_delta[0];
    RL1.point.y = RL_tf.getOrigin().y()-pc_delta[1];
    RL1.point.z = RL_tf.getOrigin().z();

    RR1.header.frame_id = "/world";
    RR1.header.stamp = ros::Time();
    RR1.point.x = RR_tf.getOrigin().x()-pc_delta[0];
    RR1.point.y = RR_tf.getOrigin().y()-pc_delta[1];
    RR1.point.z = RR_tf.getOrigin().z();

    listener.transformPoint("/base", FL1, FL1b);
    listener.transformPoint("/base", FR1, FR1b);
    listener.transformPoint("/base", RL1, RL1b);
    listener.transformPoint("/base", RR1, RR1b);

    tf::StampedTransform FRb_tf,FLb_tf,RRb_tf,RLb_tf;
    listener.lookupTransform("/base", "/FL_foot",ros::Time(0), FLb_tf);
    listener.lookupTransform("/base", "/FR_foot",ros::Time(0), FRb_tf);
    listener.lookupTransform("/base", "/RL_foot",ros::Time(0), RLb_tf);
    listener.lookupTransform("/base", "/RR_foot",ros::Time(0), RRb_tf);


    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=2*walk_base_time)){
        double u = (ros::Time::now().toSec()-init_time)/(2*walk_base_time);
        vector<double> FL_cnt_pos = {get_cubic_point(FLb_tf.getOrigin().x(),FL1b.point.x,u),
                                        get_cubic_point(FLb_tf.getOrigin().y(),FL1b.point.y,u),
                                        get_cubic_point(FLb_tf.getOrigin().z(),FL1b.point.z,u)},

                        RL_cnt_pos = {get_cubic_point(RLb_tf.getOrigin().x(),RL1b.point.x,u),
                                        get_cubic_point(RLb_tf.getOrigin().y(),RL1b.point.y,u),
                                            get_cubic_point(RLb_tf.getOrigin().z(),RL1b.point.z,u)},

                        FR_cnt_pos = {get_cubic_point(FRb_tf.getOrigin().x(),FR1b.point.x,u),
                                        get_cubic_point(FRb_tf.getOrigin().y(),FR1b.point.y,u),
                                            get_cubic_point(FRb_tf.getOrigin().z(),FR1b.point.z,u)},

                        RR_cnt_pos = {get_cubic_point(RRb_tf.getOrigin().x(),RR1b.point.x,u),
                                        get_cubic_point(RRb_tf.getOrigin().y(),RR1b.point.y,u),
                                            get_cubic_point(RRb_tf.getOrigin().z(),RR1b.point.z,u)};

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
        ros::spinOnce();
        frequency.sleep();
    }

    break;
    }
    catch (tf::TransformException ex){
    //ROS_ERROR("%s",ex.what());
    }
    }


}

void move_base3(tf::TransformListener& listener,geometry_msgs::PointStamped&  base_final_world,ros::Publisher jnt_st_pub){
    while(ros::ok()){
    try{
    ros::spinOnce();
    tf::StampedTransform FR_tf,FL_tf,RR_tf,RL_tf,base_tf;
    listener.lookupTransform("/world", "/FL_foot",ros::Time(0), FL_tf);
    listener.lookupTransform("/world", "/FR_foot",ros::Time(0), FR_tf);
    listener.lookupTransform("/world", "/RL_foot",ros::Time(0), RL_tf);
    listener.lookupTransform("/world", "/RR_foot",ros::Time(0), RR_tf);
    listener.lookupTransform("/world", "/base",ros::Time(0), base_tf);

    vector<double> pc_delta = {(FL_tf.getOrigin().x() + FR_tf.getOrigin().x()+RL_tf.getOrigin().x()+RR_tf.getOrigin().x())/4 - base_tf.getOrigin().x(),
                               (FL_tf.getOrigin().y() + FR_tf.getOrigin().y()+RL_tf.getOrigin().y()+RR_tf.getOrigin().y())/4 - base_tf.getOrigin().y()};


    geometry_msgs::PointStamped FL1,FR1,RL1,RR1;
    geometry_msgs::PointStamped FL1b,FR1b,RL1b,RR1b;

    FL1.header.frame_id = "/world";
    FL1.header.stamp = ros::Time();
    FL1.point.x = FL_tf.getOrigin().x()-pc_delta[0];
    FL1.point.y = FL_tf.getOrigin().y()-pc_delta[1];
    FL1.point.z = FL_tf.getOrigin().z();

    FR1.header.frame_id = "/world";
    FR1.header.stamp = ros::Time();
    FR1.point.x = FR_tf.getOrigin().x()-pc_delta[0];
    FR1.point.y = FR_tf.getOrigin().y()-pc_delta[1];
    FR1.point.z = FR_tf.getOrigin().z();

    RL1.header.frame_id = "/world";
    RL1.header.stamp = ros::Time();
    RL1.point.x = RL_tf.getOrigin().x()-pc_delta[0];
    RL1.point.y = RL_tf.getOrigin().y()-pc_delta[1];
    RL1.point.z = RL_tf.getOrigin().z();

    RR1.header.frame_id = "/world";
    RR1.header.stamp = ros::Time();
    RR1.point.x = RR_tf.getOrigin().x()-pc_delta[0];
    RR1.point.y = RR_tf.getOrigin().y()-pc_delta[1];
    RR1.point.z = RR_tf.getOrigin().z();

    listener.transformPoint("/base", FL1, FL1b);
    listener.transformPoint("/base", FR1, FR1b);
    listener.transformPoint("/base", RL1, RL1b);
    listener.transformPoint("/base", RR1, RR1b);

    tf::StampedTransform FRb_tf,FLb_tf,RRb_tf,RLb_tf;
    listener.lookupTransform("/base", "/FL_foot",ros::Time(0), FLb_tf);
    listener.lookupTransform("/base", "/FR_foot",ros::Time(0), FRb_tf);
    listener.lookupTransform("/base", "/RL_foot",ros::Time(0), RLb_tf);
    listener.lookupTransform("/base", "/RR_foot",ros::Time(0), RRb_tf);


    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=walk_base_time)){
        double u = (ros::Time::now().toSec()-init_time)/(walk_base_time);
        vector<double> FL_cnt_pos = {get_cubic_point(FLb_tf.getOrigin().x(),FL1b.point.x,u),
                                        get_cubic_point(FLb_tf.getOrigin().y(),FL1b.point.y,u),
                                        get_cubic_point(FLb_tf.getOrigin().z(),FL1b.point.z,u)},

                        RL_cnt_pos = {get_cubic_point(RLb_tf.getOrigin().x(),RL1b.point.x,u),
                                        get_cubic_point(RLb_tf.getOrigin().y(),RL1b.point.y,u),
                                            get_cubic_point(RLb_tf.getOrigin().z(),RL1b.point.z,u)},

                        FR_cnt_pos = {get_cubic_point(FRb_tf.getOrigin().x(),FR1b.point.x,u),
                                        get_cubic_point(FRb_tf.getOrigin().y(),FR1b.point.y,u),
                                            get_cubic_point(FRb_tf.getOrigin().z(),FR1b.point.z,u)},

                        RR_cnt_pos = {get_cubic_point(RRb_tf.getOrigin().x(),RR1b.point.x,u),
                                        get_cubic_point(RRb_tf.getOrigin().y(),RR1b.point.y,u),
                                            get_cubic_point(RRb_tf.getOrigin().z(),RR1b.point.z,u)};

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
        ros::spinOnce();
        frequency.sleep();
    }

    break;
    }
    catch (tf::TransformException ex){
    //ROS_ERROR("%s",ex.what());
    }
    }
}

void roll_and_pitch(tf::TransformListener& listener,ros::Publisher jnt_st_pub){
        while(ros::ok()){
    try{
    ros::spinOnce();
    tf::StampedTransform FR_tf,FL_tf,RR_tf,RL_tf,base_tf;
    tf::StampedTransform FR_tf_base,FL_tf_base,RR_tf_base,RL_tf_base;
    listener.lookupTransform("/world", "/FL_foot",ros::Time(0), FL_tf);
    listener.lookupTransform("/world", "/FR_foot",ros::Time(0), FR_tf);
    listener.lookupTransform("/world", "/RL_foot",ros::Time(0), RL_tf);
    listener.lookupTransform("/world", "/RR_foot",ros::Time(0), RR_tf);
    listener.lookupTransform("/world", "/base",ros::Time(0), base_tf);


    listener.lookupTransform("/base", "/FL_foot",ros::Time(0), FL_tf_base);
    listener.lookupTransform("/base", "/FR_foot",ros::Time(0), FR_tf_base);
    listener.lookupTransform("/base", "/RL_foot",ros::Time(0), RL_tf_base);
    listener.lookupTransform("/base", "/RR_foot",ros::Time(0), RR_tf_base);
    double roll =0.0;
    if(base_pose_roll<0.1) roll = 0.0;

    geometry_msgs::PointStamped FL1,FR1,RL1,RR1;
    geometry_msgs::PointStamped FL1b,FR1b,RL1b,RR1b;

    FL1.header.frame_id = "/base";
    FL1.header.stamp = ros::Time();
    FL1.point.x  = FL_tf_base.getOrigin().x();
    FL1.point.y  = FL_tf_base.getOrigin().y()*cos(roll) - FL_tf_base.getOrigin().z()*sin(roll);
    FL1.point.z  = FL_tf_base.getOrigin().y()*sin(roll) + FL_tf_base.getOrigin().z()*cos(roll);



    FR1.header.frame_id = "/base";
    FR1.header.stamp = ros::Time();
    FR1.point.x = FR_tf_base.getOrigin().x();
    FR1.point.y = FR_tf_base.getOrigin().y()*cos(roll) - FR_tf_base.getOrigin().z()*sin(roll);
    FR1.point.z = FR_tf_base.getOrigin().y()*sin(roll) + FR_tf_base.getOrigin().z()*cos(roll);
    
    RL1.header.frame_id = "/base";
    RL1.header.stamp = ros::Time();
    RL1.point.x = RL_tf_base.getOrigin().x();
    RL1.point.y  = RL_tf_base.getOrigin().y()*cos(roll) - RL_tf_base.getOrigin().z()*sin(roll);
    RL1.point.z  = RL_tf_base.getOrigin().y()*sin(roll) + RL_tf_base.getOrigin().z()*cos(roll);

    RR1.header.frame_id = "/base";
    RR1.header.stamp = ros::Time();
    RR1.point.x = RR_tf_base.getOrigin().x();
    RR1.point.y  = RR_tf_base.getOrigin().y()*cos(roll) - RR_tf_base.getOrigin().z()*sin(roll);
    RR1.point.z  = RR_tf_base.getOrigin().y()*sin(roll) + RR_tf_base.getOrigin().z()*cos(roll);




    tf::StampedTransform FR_hip_tf,FL_hip_tf,RR_hip_tf,RL_hip_tf;
    listener.lookupTransform("/FL_hip", "/FL_foot",ros::Time(0), FL_hip_tf);
    listener.lookupTransform("/FR_hip", "/FR_foot",ros::Time(0), FR_hip_tf);
    listener.lookupTransform("/RL_hip", "/RL_foot",ros::Time(0), RL_hip_tf);
    listener.lookupTransform("/RR_hip", "/RR_foot",ros::Time(0), RR_hip_tf);


    double front_height_adjust1 = 0;
    double front_height_adjust2 = 0;
    double back_height_adjust1 = 0;
    double back_height_adjust2 = 0;

    if(abs(FL_tf.getOrigin().z()-FR_tf.getOrigin().z()) < 0.03 and FL_hip_tf.getOrigin().x() < robot_base_height - 0.06){
        front_height_adjust1 = robot_base_height - 0.03 - FL_hip_tf.getOrigin().x();
        front_height_adjust2 = robot_base_height - 0.03 - FR_hip_tf.getOrigin().x();
    }
    
    if(abs(RL_tf.getOrigin().z()-RR_tf.getOrigin().z()) < 0.03 and RL_hip_tf.getOrigin().x() < robot_base_height - 0.06){
        back_height_adjust1 = robot_base_height - 0.03 - RL_hip_tf.getOrigin().x();
        back_height_adjust2 = robot_base_height -  0.03 - RR_hip_tf.getOrigin().x();
    }

    listener.transformPoint("/FL_hip", FL1, FL1b);
    listener.transformPoint("/FR_hip", FR1, FR1b);
    listener.transformPoint("/RL_hip", RL1, RL1b);
    listener.transformPoint("/RR_hip", RR1, RR1b);


    FL1b.point.x+= front_height_adjust1;
    RL1b.point.x+= back_height_adjust1;
    FR1b.point.x+= front_height_adjust2;
    RR1b.point.x+= back_height_adjust2;


    tf::StampedTransform FRb_tf,FLb_tf,RRb_tf,RLb_tf;
    listener.lookupTransform("/FL_hip", "/FL_foot",ros::Time(0), FLb_tf);
    listener.lookupTransform("/FR_hip", "/FR_foot",ros::Time(0), FRb_tf);
    listener.lookupTransform("/RL_hip", "/RL_foot",ros::Time(0), RLb_tf);
    listener.lookupTransform("/RR_hip", "/RR_foot",ros::Time(0), RRb_tf);


    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=walk_base_time)){
        double u = (ros::Time::now().toSec()-init_time)/(walk_base_time);
        vector<double> FL_cnt_pos = {get_cubic_point(FLb_tf.getOrigin().x(),FL1b.point.x,u),
                                        get_cubic_point(FLb_tf.getOrigin().y(),FL1b.point.y,u),
                                        get_cubic_point(FLb_tf.getOrigin().z(),FL1b.point.z,u)},

                        RL_cnt_pos = {get_cubic_point(RLb_tf.getOrigin().x(),RL1b.point.x,u),
                                        get_cubic_point(RLb_tf.getOrigin().y(),RL1b.point.y,u),
                                            get_cubic_point(RLb_tf.getOrigin().z(),RL1b.point.z,u)},

                        FR_cnt_pos = {get_cubic_point(FRb_tf.getOrigin().x(),FR1b.point.x,u),
                                        get_cubic_point(FRb_tf.getOrigin().y(),FR1b.point.y,u),
                                            get_cubic_point(FRb_tf.getOrigin().z(),FR1b.point.z,u)},

                        RR_cnt_pos = {get_cubic_point(RRb_tf.getOrigin().x(),RR1b.point.x,u),
                                        get_cubic_point(RRb_tf.getOrigin().y(),RR1b.point.y,u),
                                            get_cubic_point(RRb_tf.getOrigin().z(),RR1b.point.z,u)};

        vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(FL_cnt_pos),
                        RR_req_jnt = quad_kinem_g.Right_Leg_IK(RR_cnt_pos),
                        FR_req_jnt = quad_kinem_g.Right_Leg_IK(FR_cnt_pos),
                        RL_req_jnt = quad_kinem_g.Left_Leg_IK(RL_cnt_pos);
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
        ros::spinOnce();
        frequency.sleep();
    }

    break;
    }
    catch (tf::TransformException ex){
    //ROS_ERROR("%s",ex.what());
    }
    }
}
void adjust_pitch(ros::Publisher jnt_st_pub){
    vector<double> FL_init = current_robot_footsteps[0],FR_init = current_robot_footsteps[1], // change to local base
                RL_init = current_robot_footsteps[2],RR_init = current_robot_footsteps[3];
    vector<double> FL_cnt_pos = current_robot_footsteps[0],FR_cnt_pos = current_robot_footsteps[1], // change to local base
                RL_cnt_pos = current_robot_footsteps[2],RR_cnt_pos = current_robot_footsteps[3];
    bool front = false,back=false;
    if(abs(FL_init[2])<(robot_base_height-0.01) && abs(FR_init[2])<(robot_base_height-0.01) &&  abs(FL_init[2]-FR_init[2])<0.04) front = true;
    if(abs(RL_init[2])<(robot_base_height-0.01) && abs(RR_init[2])<(robot_base_height-0.01) &&  abs(RL_init[2]-RR_init[2])<0.04) back = true;
    
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    double T = max(  abs(FL_init[2]+robot_base_height)/robot_verti_vel,  abs(RL_init[2]+robot_base_height)/robot_verti_vel);
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T) && (front || back)){
        double u = (ros::Time::now().toSec()-init_time)/T;
        if(front){
            FL_cnt_pos[2] = FL_init[2]*(1-u) -(robot_base_height)*u;
            FR_cnt_pos[2] = FR_init[2]*(1-u) -(robot_base_height)*u;
            vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(FL_cnt_pos)),
                    FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(FR_cnt_pos));
            jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
            jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
            jnt_set_st.joint_positions[2] = -FR_req_jnt[2];
            jnt_set_st.joint_positions[3] = FL_req_jnt[0];
            jnt_set_st.joint_positions[4] = FL_req_jnt[1];
            jnt_set_st.joint_positions[5] = FL_req_jnt[2];
            jnt_st_pub.publish(jnt_set_st);
            ros::spinOnce();
            frequency.sleep();
        }
        if(back){
            RL_cnt_pos[2] = RL_init[2]*(1-u) -(robot_base_height)*u;
            RR_cnt_pos[2] = RR_init[2]*(1-u) -(robot_base_height)*u;
            vector<double> RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(RL_cnt_pos)),
                    RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(RR_cnt_pos));
            jnt_set_st.joint_positions[9] = RL_req_jnt[0];
            jnt_set_st.joint_positions[10]= RL_req_jnt[1];
            jnt_set_st.joint_positions[11]= RL_req_jnt[2];
            jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
            jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
            jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
            jnt_st_pub.publish(jnt_set_st);
            ros::spinOnce();
            frequency.sleep();
        }
    }
}
void adjust_roll(ros::Publisher jnt_st_pub){
    vector<double> FL_init = current_robot_footsteps[0],FR_init = current_robot_footsteps[1], // change to local base
                RL_init = current_robot_footsteps[2],RR_init = current_robot_footsteps[3];
    vector<double> FL_cnt_pos = current_robot_footsteps[0],FR_cnt_pos = current_robot_footsteps[1], // change to local base
                RL_cnt_pos = current_robot_footsteps[2],RR_cnt_pos = current_robot_footsteps[3];
    if(base_pose_roll !=0){
        double delta = sin(base_pose_roll)*((robot_config[1] + 2*robot_config[2]))/2;
        quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        double T = 0.45;
        while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T)){
            double u = (ros::Time::now().toSec()-init_time)/T;
                FL_cnt_pos[2] = FL_init[2]*(1-u) +(FL_init[2]+delta)*u;
                FR_cnt_pos[2] = FR_init[2]*(1-u) +( FR_init[2]-delta)*u;
                vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(FL_cnt_pos)),
                        FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(FR_cnt_pos));
                jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
                jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
                jnt_set_st.joint_positions[2] = -FR_req_jnt[2];
                jnt_set_st.joint_positions[3] = FL_req_jnt[0];
                jnt_set_st.joint_positions[4] = FL_req_jnt[1];
                jnt_set_st.joint_positions[5] = FL_req_jnt[2];
                RL_cnt_pos[2] = RL_init[2]*(1-u) +(RL_init[2]+delta)*u;
                RR_cnt_pos[2] = RR_init[2]*(1-u) +(RR_init[2]-delta)*u;
                vector<double> RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(RL_cnt_pos)),
                        RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(RR_cnt_pos));
                jnt_set_st.joint_positions[9] = RL_req_jnt[0];
                jnt_set_st.joint_positions[10]= RL_req_jnt[1];
                jnt_set_st.joint_positions[11]= RL_req_jnt[2];
                jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
                jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
                jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
                jnt_st_pub.publish(jnt_set_st);
                ros::spinOnce();
                frequency.sleep();
    }
    }
}
void swng(ros::Publisher jnt_st_pub,vector<double>& iniat,vector<double> finl,int cls){
    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(controller_rate);
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
        vector<vector<double>> A = generate_swing_coefs_walk(iniat,finl);
    vector<double> cnt_pos,FL_req_jnt,FR_req_jnt,RL_req_jnt,RR_req_jnt;
    
    tf::TransformListener listener;
    while(ros::ok()&&(ros::Time::now().toSec()-init_time<=swing_time)){
        double u = (ros::Time::now().toSec()-init_time)/swing_time;
        vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
        switch (cls)
        {
        case 0:
            if(contacts[0]==0  || u<0.2){
            cnt_pos = Multiply(u_mat,A)[0];
            FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(cnt_pos));
            jnt_set_st.joint_positions[3] = FL_req_jnt[0];
            jnt_set_st.joint_positions[4] = FL_req_jnt[1];
            jnt_set_st.joint_positions[5] = FL_req_jnt[2];
            jnt_st_pub.publish(jnt_set_st);        
            }
            break;
        case 1:
            if(contacts[1]==0  || u<0.2){ 
            cnt_pos = Multiply(u_mat,A)[0];
            FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(cnt_pos));
            jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
            jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
            jnt_set_st.joint_positions[2] = -FR_req_jnt[2];
            jnt_st_pub.publish(jnt_set_st);
            }        
            break;
        case 2:
            if(contacts[2]==0  || u<0.2){ cnt_pos = Multiply(u_mat,A)[0];
            RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(cnt_pos));
            jnt_set_st.joint_positions[9] = RL_req_jnt[0];
            jnt_set_st.joint_positions[10]= RL_req_jnt[1];
            jnt_set_st.joint_positions[11]= RL_req_jnt[2];
            jnt_st_pub.publish(jnt_set_st);        
            }
            break;
        case 3:
            if(contacts[3]==0  || u<0.2){
            cnt_pos = Multiply(u_mat,A)[0];
            RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(cnt_pos));
            jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
            jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
            jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
            jnt_st_pub.publish(jnt_set_st);        
            }
            break;
        default:
            break;
        }        
        frequency.sleep();
        ros::spinOnce();
    }


    // geometry_msgs::PointStamped base_point;
    // base_point.header.frame_id = "base";
    // base_point.header.stamp = ros::Time();
    // base_point.point.x = cnt_pos[0];
    // base_point.point.y = cnt_pos[1];
    // base_point.point.z = cnt_pos[2];
    // geometry_msgs::PointStamped world_point;
    // listener.transformPoint("/world", base_point, world_point);
    // world_point.point.z-=0.23;
    // listener.transformPoint("/base", world_point, base_point);


    // init_time = ros::Time::now().toSec();
    // bool nd = true;
    // vector<double> cnt_pos1 = cnt_pos;
    // while(ros::ok()&&(ros::Time::now().toSec()-init_time<=swing_time)&&nd){
    //     double u = (ros::Time::now().toSec()-init_time)/swing_time;
    //     switch (cls)
    //     {
    //     case 0:
    //         if(contacts[0]==0){
    //         cnt_pos[0] = cnt_pos1[0]*(1-u) + u*base_point.point.x;
    //         cnt_pos[1] = cnt_pos1[1]*(1-u) + u*base_point.point.y;
    //         cnt_pos[2] = cnt_pos1[2]*(1-u) + u*base_point.point.z;
    //         FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(cnt_pos));
    //         jnt_set_st.joint_positions[3] = FL_req_jnt[0];
    //         jnt_set_st.joint_positions[4] = FL_req_jnt[1];
    //         jnt_set_st.joint_positions[5] = FL_req_jnt[2];
    //         jnt_st_pub.publish(jnt_set_st);        
    //         }
    //         else {
    //             nd = false;
    //         }
    //         break;
    //     case 1:
    //         if(contacts[1]==0){ 
    //         cnt_pos[0] = cnt_pos1[0]*(1-u) + u*base_point.point.x;
    //         cnt_pos[1] = cnt_pos1[1]*(1-u) + u*base_point.point.y;
    //         cnt_pos[2] = cnt_pos1[2]*(1-u) + u*base_point.point.z;
    //         FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(cnt_pos));
    //         jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
    //         jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
    //         jnt_set_st.joint_positions[2] = -FR_req_jnt[2];
    //         jnt_st_pub.publish(jnt_set_st);
    //         } 
    //         else {
    //             nd = false;
    //         }      
    //         break;
    //     case 2:
    //         if(contacts[2]==0){
    //         cnt_pos[0] = cnt_pos1[0]*(1-u) + u*base_point.point.x;
    //         cnt_pos[1] = cnt_pos1[1]*(1-u) + u*base_point.point.y;
    //         cnt_pos[2] = cnt_pos1[2]*(1-u) + u*base_point.point.z;
    //         RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(cnt_pos));
    //         jnt_set_st.joint_positions[9] = RL_req_jnt[0];
    //         jnt_set_st.joint_positions[10]= RL_req_jnt[1];
    //         jnt_set_st.joint_positions[11]= RL_req_jnt[2];
    //         jnt_st_pub.publish(jnt_set_st);        
    //         }
    //         else {
    //             nd = false;
    //         }
    //         break;
    //     case 3:
    //         if(contacts[3]==0){
    //         cnt_pos[0] = cnt_pos1[0]*(1-u) + u*base_point.point.x;
    //         cnt_pos[1] = cnt_pos1[1]*(1-u) + u*base_point.point.y;
    //         cnt_pos[2] = cnt_pos1[2]*(1-u) + u*base_point.point.z;
    //         RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(cnt_pos));
    //         jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
    //         jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
    //         jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
    //         jnt_st_pub.publish(jnt_set_st);        
    //         }
    //         else {
    //             nd = false;
    //         }
    //         break;
    //     default:
    //         break;
    //     }        
    //     frequency.sleep();
    //     ros::spinOnce();
    // }

    iniat = cnt_pos;


}

void rotate_base(ros::Publisher jnt_st_pub,vector<double>&  FL_init,vector<double>&  FR_init,vector<double>&  RL_init,vector<double>&  RR_init,double omega){
        vector<double> FL = {FL_init[0]*cos(omega) - FL_init[1]*sin(omega) - FL_init[0] ,FL_init[0]*sin(omega) + FL_init[1]*cos(omega) - FL_init[1]},
                        RL = {RL_init[0]*cos(omega) - RL_init[1]*sin(omega) - RL_init[0] ,RL_init[0]*sin(omega) + RL_init[1]*cos(omega) - RL_init[1]},
                        FR = {FR_init[0]*cos(omega) - FR_init[1]*sin(omega) - FR_init[0] ,FR_init[0]*sin(omega) + FR_init[1]*cos(omega) - FR_init[1]},
                        RR = {RR_init[0]*cos(omega) - RR_init[1]*sin(omega) - RR_init[0] ,RR_init[0]*sin(omega) + RR_init[1]*cos(omega) - RR_init[1]};
        double T = 1.0;
        quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
        double init_time = ros::Time::now().toSec();
        ros::Rate frequency(controller_rate);
        while(ros::ok()&&(ros::Time::now().toSec()-init_time<=T)){
            double u = (ros::Time::now().toSec()-init_time)/T;
            vector<double> FL_cnt_pos = {FL_init[0]-FL[0]*u, FL_init[1]-FL[1]*u, FL_init[2]},
                            RL_cnt_pos = {RL_init[0]-RL[0]*u, RL_init[1]-RL[1]*u, RL_init[2]},
                            FR_cnt_pos = {FR_init[0]-FR[0]*u, FR_init[1]-FR[1]*u, FR_init[2]},
                            RR_cnt_pos = {RR_init[0]-RR[0]*u, RR_init[1]-RR[1]*u, RR_init[2]};

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
            ros::spinOnce();
            frequency.sleep();
        }
}

void walk_a_step(ros::Publisher jnt_st_pub,tf::TransformListener& listener){

    if(foot_holds.vel1==0 && foot_holds.vel2==0) return;
    vector<double> FL_init = current_robot_footsteps[0],FR_init = current_robot_footsteps[1], // change to local base
                    RL_init = current_robot_footsteps[2],RR_init = current_robot_footsteps[3];
    double X = foot_holds.base_pose.position.x,Y = foot_holds.base_pose.position.y,omega =foot_holds.base_pose.orientation.z ;
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);

    vector<double> FL1 = {foot_holds.FL1.x,foot_holds.FL1.y,foot_holds.FL1.z+toe_radius},FR1 = {foot_holds.FR1.x,foot_holds.FR1.y,foot_holds.FR1.z+toe_radius/2},
                RL1 = {foot_holds.RL1.x,foot_holds.RL1.y,foot_holds.RL1.z+toe_radius},RR1 = {foot_holds.RR1.x,foot_holds.RR1.y,foot_holds.RR1.z+toe_radius/2};

    ros::spinOnce();

    


    geometry_msgs::PointStamped base_final_point;
    base_final_point.header.frame_id = "world";
    base_final_point.header.stamp = ros::Time();
    base_final_point.point.x = X;
    base_final_point.point.y = Y;
    base_final_point.point.z = 0;


    geometry_msgs::PointStamped FL1_point,FR1_point,RL1_point,RR1_point;
    geometry_msgs::PointStamped FL1b_point,FR1b_point,RL1b_point,RR1b_point;
    FL1_point.header.frame_id = "/world";
    FL1_point.header.stamp = ros::Time();
    FL1_point.point.x = FL1[0];
    FL1_point.point.y = FL1[1];
    FL1_point.point.z = FL1[2];

    FR1_point.header.frame_id = "/world";
    FR1_point.header.stamp = ros::Time();
    FR1_point.point.x = FR1[0];
    FR1_point.point.y = FR1[1];
    FR1_point.point.z = FR1[2];

    RL1_point.header.frame_id = "/world";
    RL1_point.header.stamp = ros::Time();
    RL1_point.point.x = RL1[0];
    RL1_point.point.y = RL1[1];
    RL1_point.point.z = RL1[2];

    RR1_point.header.frame_id = "/world";
    RR1_point.header.stamp = ros::Time();
    RR1_point.point.x = RR1[0];
    RR1_point.point.y = RR1[1];
    RR1_point.point.z = RR1[2];

    ros::spinOnce();

    ros::Duration(0.1).sleep();
    FL_init = current_robot_footsteps[0];FR_init = current_robot_footsteps[1]; // change to local base
                    RL_init = current_robot_footsteps[2];RR_init = current_robot_footsteps[3];
    if(abs(omega)>0.005) rotate_base(jnt_st_pub,FL_init,FR_init,RL_init,RR_init,omega);  

    move_base1(listener,RL1_point,jnt_st_pub);

    listener.transformPoint("/base", RL1_point, RL1b_point);
    RL1 = {RL1b_point.point.x,RL1b_point.point.y,RL1b_point.point.z};
    swng(jnt_st_pub,current_robot_footsteps[2],RL1,2);

    listener.transformPoint("/base", FL1_point, FL1b_point);
    FL1 = {FL1b_point.point.x,FL1b_point.point.y,FL1b_point.point.z};
    swng(jnt_st_pub,current_robot_footsteps[0],FL1,0);


    ros::Duration(0.1).sleep();
    move_base2(listener,RR1_point,jnt_st_pub);

   
    listener.transformPoint("/base", RR1_point, RR1b_point);
    RR1 = {RR1b_point.point.x,RR1b_point.point.y,RR1b_point.point.z};
    swng(jnt_st_pub,current_robot_footsteps[3],RR1,3);

    listener.transformPoint("/base", FR1_point, FR1b_point);
    FR1 = {FR1b_point.point.x,FR1b_point.point.y,FR1b_point.point.z};
    swng(jnt_st_pub,current_robot_footsteps[1],FR1,1);
    


    ros::Duration(0.05).sleep();
    move_base3(listener,base_final_point,jnt_st_pub);
    if(robot_name == "dogbot"){
        std::cout<<"working"<<std::endl;
        roll_and_pitch(listener,jnt_st_pub);
    }
    else{
            adjust_pitch(jnt_st_pub);
            adjust_roll(jnt_st_pub);

    }

}
