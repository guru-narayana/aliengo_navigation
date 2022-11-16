#include "aliengo_footstep_planner/quad_footstep_planner.h"


double get_edge_cost(Position center,double radius)
{
  double cost = 0;
  for (grid_map::CircleIterator iterator(elev_map, center, radius);
      !iterator.isPastEnd(); ++iterator) {
    cost+=elev_map.at("normal_vectors_z", *iterator);
  }
    return cost;
}

vector<vector<double>> next_step(double R,double theta,double v,double w,double base_x,double base_y,double yaw,vector<double> Position_0,int foot_type){
    vector<double> Foot_1(3,0),temp(2,0),Foot_2(3,0);
    double SL = min_steplength,alpha=0,min_cost(100000.0);
    vector<vector<double>> out ={};
    vector<double> vr_vect={0};
    while(SL<max_steplength){
        SL += resolution_steplength;
        if(v==0) SL ==0.03;
        alpha = 2*asin(SL/(2*R));
        double theta0(0);
        switch(foot_type){
            case 0://FL_foot
                if(w<0){
                temp[0] = R*sin(theta+alpha);
                temp[1] = R*cos(theta+alpha);
                }
                else{
                temp[0] = R*sin(theta-alpha);
                temp[1] = R*cos(theta-alpha);            
                }
                theta0 = atan((temp[0]-Position_0[0])/(temp[1]-Position_0[1]));
                break;
            case 1://FR_foot
                if(w<0){
                temp[0] = R*sin(theta-alpha);
                temp[1] = -R*cos(theta-alpha);
                }
                else{
                temp[0] = R*sin(theta+alpha);
                temp[1] = -R*cos(theta+alpha);            
                }
                theta0 = atan((temp[0]-Position_0[0])/(temp[1]-Position_0[1]));
                break;
            case 2://RR_foot
                if(w<0){
                temp[0] = -R*sin(theta+alpha);
                temp[1] = -R*cos(theta+alpha);
                theta0 =  M_PI - atan(abs(temp[0]-Position_0[0])/abs(temp[1]-Position_0[1]));
                }
                else{
                temp[0] = -R*sin(theta-alpha);
                temp[1] = -R*cos(theta-alpha);            
                theta0 = M_PI - atan(abs(temp[0]-Position_0[0])/abs(temp[1]-Position_0[1]));
                }
                break;
            case 3://RL_foot
                if(w<0){
                temp[0] = -R*sin(theta-alpha);
                temp[1] = R*cos(theta-alpha);
                theta0 =  M_PI + atan(abs(temp[0]-Position_0[0])/abs(temp[1]-Position_0[1]));;
                }
                else{
                temp[0] = -R*sin(theta+alpha);
                temp[1] = R*cos(theta+alpha);            
                theta0 =  M_PI + atan(abs(temp[0]-Position_0[0])/abs(temp[1]-Position_0[1]));
                }
                break;
            default:
                break;
        }
        double vr = (SL*w)/alpha;
        double theta1 = atan2((v + vr*sin(theta0)),(vr*cos(theta0)));
        double x = Position_0[0] + SL*sin(theta1);
        double y = Position_0[1] + SL*cos(theta1);
        double x1 = cos(yaw)*(x+base_x) - sin(yaw)*(y+base_y) + base_pose_x;
        double y1 = sin(yaw)*(x+base_x) + cos(yaw)*(y+base_y) + base_pose_y;
        double height = elev_map.atPosition("elevation_inpainted",Position(x1,y1));
        double surf_normal = get_edge_cost(Position(x1,y1),0.04);
        double cost = height*obstacle_stepcostFactor + abs(SL-favoured_steplength)*prefered_stepcostFactor - surf_normal*edge_costFactor;
        cout<<cost<<endl;

        if(cost<min_cost){
            Foot_1 = {x1,y1,height};
            min_cost = cost;
            vr_vect[0] = sqrt(pow((v + vr*sin(theta0)),2) + pow((vr*cos(theta0)),2));
        }
    }
    out.push_back(Foot_1);
    out.push_back(vr_vect);

    return out;
}

void get_optim_vels(){
    double velx_incrm = max_forward_vel/(vx_samples-1),
            veltheta_incrm = (max_angular_vel-min_angular_vel)/(vtheta_samples-1),
            min_cost = 100000.0,optim_velx=0,optim_vely=0;
    for(int vx_cnt = 0;vx_cnt<vx_samples;vx_cnt++){
        for(int vt_cnt = 0;vt_cnt<vtheta_samples ;vt_cnt++){
            joystick_vals[1] = (vx_cnt*velx_incrm)/max_forward_vel;
            joystick_vals[0] = (min_angular_vel + vt_cnt*veltheta_incrm)/max_angular_vel;
            double cost = collision_check(true);
            if(min_cost>cost){
                min_cost = cost;
                optim_velx = joystick_vals[1];
                optim_vely = joystick_vals[0];
            }
        }
    }
    joystick_vals[0] = optim_vely;
    joystick_vals[1] = optim_velx;
}

double get_globalPlan_cost(double x,double y,int sample_points){
    double min_distance = 10000;
    int val = min(sample_points , int(get_plan.response.plan.poses.size()));
    for(int i=0; i<val;i++){
        double distance = sqrt(pow((get_plan.response.plan.poses[i].pose.position.x - x),2)+
                                pow((get_plan.response.plan.poses[i].pose.position.y - y),2));
        if(distance<min_distance){
            min_distance = distance;
        }
    }
    return min_distance;
}


void plan_footsteps(ros::Publisher poly_pub,ros::Publisher foot_marker_pub,ros::Publisher next_step_pub){
    aliengo_msgs::transition_foothold transn_footholds;
    if(using_joystick && (joystick_vals[0]!=0 || joystick_vals[1]!=0)){
        double collision_cost = collision_check(false);
        // if(collision_cost>0.0){
        //     transn_footholds.vel1 = 0.0;
        //     transn_footholds.vel2 = 0.0;
        //     next_step_pub.publish(transn_footholds);
        //     return;
        // }
    }
    else if(!using_joystick && recived_global_plan){
        // cout<<global_path_poses[0].pose.position.x<<endl;
        get_optim_vels();
    }
    else{
        transn_footholds.vel1 = 0.0;
        transn_footholds.vel2 = 0.0;
        next_step_pub.publish(transn_footholds);
        return;
    }
    transn_footholds.collision_halt = false;
    double length (robot_config[0]*cos(base_pose_pitch)), 
            width ((robot_config[1] + 2*robot_config[2]));

    double  v (max(0.0,(joystick_vals[1]*max_forward_vel))),
            w (joystick_vals[0]*max_angular_vel);

    double R(sqrt(pow(length/2,2)+pow(width/2,2))),
            theta1(atan(length/width));
    if(w!=0 || v!=0){
        double time_prd(0);
        if (w==0 && v!=0) time_prd = abs((steps_horizon*favoured_steplength)/v);
        else if(w!=0 && abs(v/w)<=(width*2/2.2)) time_prd = abs(2*steps_horizon*asin(favoured_steplength/(2*R))/w);
        else if(w!=0 && abs(v/w)>(width*2/2.2)) time_prd = abs(2*steps_horizon*asin((favoured_steplength*w)/(2*v))/w);
        vector<double>  FL_0 = {length/2,width/2},FR_0 = {length/2,-width/2},
                        RR_0 = {-length/2,-width/2},RL_0 = {-length/2,width/2};
        vector<vector<double>> FL_foot_holds1,FR_foot_holds1,RL_foot_holds1,RR_foot_holds1;

        for(int i=0;i<steps_horizon;i++){
            double t = (time_prd/steps_horizon)*i;
            double base_x(0),base_y(0),
                    beta = base_pose_yaw + w*t;
            if(w != 0){
                base_x = (v/w)*sin(w*t);
                base_y = -(v/w)*(cos(w*t)-1);
            }
            else{
                base_x = v*t;
                base_y = 0;
            }
            if(i == 1){
                transn_footholds.base_pose.position.x = base_pose_x + cos(-base_pose_yaw)*(base_x) - sin(-base_pose_yaw)*(base_y);
                transn_footholds.base_pose.position.y = base_pose_y + sin(-base_pose_yaw)*(base_x) + cos(-base_pose_yaw)*(base_y);
                transn_footholds.base_pose.orientation.z = w*t;
            }
            FL_foot_holds1.push_back(next_step(R, theta1, v, w, base_x, base_y, beta, FL_0,0)[0]);
            FR_foot_holds1.push_back(next_step(R, theta1, v, w, base_x, base_y, beta, FR_0,1)[0]);
            RR_foot_holds1.push_back(next_step(R, theta1, v, w, base_x, base_y, beta, RR_0,2)[0]);
            RL_foot_holds1.push_back(next_step(R, theta1, v, w, base_x, base_y, beta, RL_0,3)[0]);
        }
        double planar_cost(0);
        for(int i=0;i<steps_horizon;i++){
        planar_cost+= (FL_foot_holds1[i][2]+FR_foot_holds1[i][2])/2;
        }
        
        transn_footholds.Future_planarcost = planar_cost/steps_horizon;
        transn_footholds.FL1.x = FL_foot_holds1[0][0];
        transn_footholds.FL1.y = FL_foot_holds1[0][1];
        transn_footholds.FL1.z = FL_foot_holds1[0][2];
        transn_footholds.FR1.x = FR_foot_holds1[0][0];
        transn_footholds.FR1.y = FR_foot_holds1[0][1];
        transn_footholds.FR1.z = FR_foot_holds1[0][2];
        transn_footholds.RL1.x = RL_foot_holds1[0][0];
        transn_footholds.RL1.y = RL_foot_holds1[0][1];
        transn_footholds.RL1.z = RL_foot_holds1[0][2];
        transn_footholds.RR1.x = RR_foot_holds1[0][0];
        transn_footholds.RR1.y = RR_foot_holds1[0][1];
        transn_footholds.RR1.z = RR_foot_holds1[0][2];
        transn_footholds.FL2.x = FL_foot_holds1[1][0];
        transn_footholds.FL2.y = FL_foot_holds1[1][1];
        transn_footholds.FL2.z = FL_foot_holds1[1][2];
        transn_footholds.FR2.x = FR_foot_holds1[1][0];
        transn_footholds.FR2.y = FR_foot_holds1[1][1];
        transn_footholds.FR2.z = FR_foot_holds1[1][2];
        transn_footholds.RL2.x = RL_foot_holds1[1][0];
        transn_footholds.RL2.y = RL_foot_holds1[1][1];
        transn_footholds.RL2.z = RL_foot_holds1[1][2];
        transn_footholds.RR2.x = RR_foot_holds1[1][0];
        transn_footholds.RR2.y = RR_foot_holds1[1][1];
        transn_footholds.RR2.z = RR_foot_holds1[1][2];

        transn_footholds.vel1 = (max(0.0,(joystick_vals[1]*max_forward_vel)));
        transn_footholds.vel2 = (joystick_vals[0]*max_angular_vel);
        next_step_pub.publish(transn_footholds);

        if(visualize_plan){
            visualization_msgs::MarkerArray Foot_Markerarr;
            Foot_Markerarr.markers.resize(steps_horizon*4);
            for(int i=0;i<steps_horizon;i++){
                Foot_Markerarr.markers[i].header.frame_id = "map";
                Foot_Markerarr.markers[i].header.stamp = ros::Time::now();
                Foot_Markerarr.markers[i].ns = "foots";
                Foot_Markerarr.markers[i].id = i;
                Foot_Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
                Foot_Markerarr.markers[i].type = visualization_msgs::Marker::SPHERE;
                Foot_Markerarr.markers[i].pose.position.x =  FL_foot_holds1[i][0];
                Foot_Markerarr.markers[i].pose.position.y =  FL_foot_holds1[i][1];
                Foot_Markerarr.markers[i].pose.position.z =  FL_foot_holds1[i][2];
                Foot_Markerarr.markers[i].pose.orientation.x = 0.0;
                Foot_Markerarr.markers[i].pose.orientation.y = 0.0;
                Foot_Markerarr.markers[i].pose.orientation.z = 0.0;
                Foot_Markerarr.markers[i].pose.orientation.w = 1.0;
                Foot_Markerarr.markers[i].scale.x = 0.03;
                Foot_Markerarr.markers[i].scale.y = 0.03;
                Foot_Markerarr.markers[i].scale.z = 0.03;
                Foot_Markerarr.markers[i].color.a = 1.0;
                Foot_Markerarr.markers[i].color.r = 0.0;
                Foot_Markerarr.markers[i].color.g = 1.0;
                Foot_Markerarr.markers[i].color.b = 0.0;
                Foot_Markerarr.markers[i+steps_horizon].header.frame_id = "map";
                Foot_Markerarr.markers[i+steps_horizon].header.stamp = ros::Time::now();
                Foot_Markerarr.markers[i+steps_horizon].ns = "foots";
                Foot_Markerarr.markers[i+steps_horizon].id = i+steps_horizon;
                Foot_Markerarr.markers[i+steps_horizon].action = visualization_msgs::Marker::ADD;
                Foot_Markerarr.markers[i+steps_horizon].type = visualization_msgs::Marker::SPHERE;
                Foot_Markerarr.markers[i+steps_horizon].pose.position.x =  FR_foot_holds1[i][0];
                Foot_Markerarr.markers[i+steps_horizon].pose.position.y =  FR_foot_holds1[i][1];
                Foot_Markerarr.markers[i+steps_horizon].pose.position.z =  FR_foot_holds1[i][2];
                Foot_Markerarr.markers[i+steps_horizon].pose.orientation.x = 0.0;
                Foot_Markerarr.markers[i+steps_horizon].pose.orientation.y = 0.0;
                Foot_Markerarr.markers[i+steps_horizon].pose.orientation.z = 0.0;
                Foot_Markerarr.markers[i+steps_horizon].pose.orientation.w = 1.0;
                Foot_Markerarr.markers[i+steps_horizon].scale.x = 0.03;
                Foot_Markerarr.markers[i+steps_horizon].scale.y = 0.03;
                Foot_Markerarr.markers[i+steps_horizon].scale.z = 0.03;
                Foot_Markerarr.markers[i+steps_horizon].color.a = 1.0;
                Foot_Markerarr.markers[i+steps_horizon].color.r = 1.0;
                Foot_Markerarr.markers[i+steps_horizon].color.g = 0.0;
                Foot_Markerarr.markers[i+steps_horizon].color.b = 0.0;
                Foot_Markerarr.markers[i+2*steps_horizon].header.frame_id = "map";
                Foot_Markerarr.markers[i+2*steps_horizon].header.stamp = ros::Time::now();
                Foot_Markerarr.markers[i+2*steps_horizon].ns = "foots";
                Foot_Markerarr.markers[i+2*steps_horizon].id = i+steps_horizon*2;
                Foot_Markerarr.markers[i+2*steps_horizon].action = visualization_msgs::Marker::ADD;
                Foot_Markerarr.markers[i+2*steps_horizon].type = visualization_msgs::Marker::SPHERE;
                Foot_Markerarr.markers[i+2*steps_horizon].pose.position.x =  RL_foot_holds1[i][0];
                Foot_Markerarr.markers[i+2*steps_horizon].pose.position.y =  RL_foot_holds1[i][1];
                Foot_Markerarr.markers[i+2*steps_horizon].pose.position.z =  RL_foot_holds1[i][2];
                Foot_Markerarr.markers[i+2*steps_horizon].pose.orientation.x = 0.0;
                Foot_Markerarr.markers[i+2*steps_horizon].pose.orientation.y = 0.0;
                Foot_Markerarr.markers[i+2*steps_horizon].pose.orientation.z = 0.0;
                Foot_Markerarr.markers[i+2*steps_horizon].pose.orientation.w = 1.0;
                Foot_Markerarr.markers[i+2*steps_horizon].scale.x = 0.03;
                Foot_Markerarr.markers[i+2*steps_horizon].scale.y = 0.03;
                Foot_Markerarr.markers[i+2*steps_horizon].scale.z = 0.03;
                Foot_Markerarr.markers[i+2*steps_horizon].color.a = 1.0;
                Foot_Markerarr.markers[i+2*steps_horizon].color.r = 0.0;
                Foot_Markerarr.markers[i+2*steps_horizon].color.g = 0.0;
                Foot_Markerarr.markers[i+2*steps_horizon].color.b = 1.0;
                Foot_Markerarr.markers[i+3*steps_horizon].header.frame_id = "map";
                Foot_Markerarr.markers[i+3*steps_horizon].header.stamp = ros::Time::now();
                Foot_Markerarr.markers[i+3*steps_horizon].ns = "foots";
                Foot_Markerarr.markers[i+3*steps_horizon].id = i+steps_horizon*3;
                Foot_Markerarr.markers[i+3*steps_horizon].action = visualization_msgs::Marker::ADD;
                Foot_Markerarr.markers[i+3*steps_horizon].type = visualization_msgs::Marker::SPHERE;
                Foot_Markerarr.markers[i+3*steps_horizon].pose.position.x =  RR_foot_holds1[i][0];
                Foot_Markerarr.markers[i+3*steps_horizon].pose.position.y =  RR_foot_holds1[i][1];
                Foot_Markerarr.markers[i+3*steps_horizon].pose.position.z =  RR_foot_holds1[i][2];
                Foot_Markerarr.markers[i+3*steps_horizon].pose.orientation.x = 0.0;
                Foot_Markerarr.markers[i+3*steps_horizon].pose.orientation.y = 0.0;
                Foot_Markerarr.markers[i+3*steps_horizon].pose.orientation.z = 0.0;
                Foot_Markerarr.markers[i+3*steps_horizon].pose.orientation.w = 1.0;
                Foot_Markerarr.markers[i+3*steps_horizon].scale.x = 0.03;
                Foot_Markerarr.markers[i+3*steps_horizon].scale.y = 0.03;
                Foot_Markerarr.markers[i+3*steps_horizon].scale.z = 0.03;
                Foot_Markerarr.markers[i+3*steps_horizon].color.a = 1.0;
                Foot_Markerarr.markers[i+3*steps_horizon].color.r = 1.0;
                Foot_Markerarr.markers[i+3*steps_horizon].color.g = 0.0;
                Foot_Markerarr.markers[i+3*steps_horizon].color.b = 0.0;
            }
            foot_marker_pub.publish(Foot_Markerarr);
        }   
    }
}

double collision_check(bool return_totalcost){
    int sample_points = (int)(horizon_length*max_steplength/grid_map_resolution);
    double collision_cost=0;
    double v = max(0.0,(joystick_vals[1]*max_forward_vel));
    double w = (joystick_vals[0]*max_angular_vel);
    double x1,x2,x3,x4,y1,y2,y3,y4,x,y,fx,fy;
    double time_period = (max_steplength*horizon_length)/max_forward_vel;
    int total_points = 1;
    int collision_points = 0;
    for (int i=1;i<=sample_points;i++){
        bool collision(false);
        double t = i*time_period/sample_points;
        if(w != 0){
            x = (v/w)*sin(w*t);
            y = -(v/w)*(cos(w*t)-1);
        }
        else{
            x = v*t;
            y = 0;
        }
        double beta = w*t + base_pose_yaw; 
        x1 = base_pose_x + cos(beta)*(x+collision_rect_length/2) - sin(beta)*(y+collision_rect_width/2);
        y1 = base_pose_y + sin(beta)*(x+collision_rect_length/2) + cos(beta)*(y+collision_rect_width/2);
        x2 = base_pose_x + cos(beta)*(x-collision_rect_length/2) - sin(beta)*(y+collision_rect_width/2);
        y2 = base_pose_y + sin(beta)*(x-collision_rect_length/2) + cos(beta)*(y+collision_rect_width/2);
        x3 = base_pose_x + cos(beta)*(x-collision_rect_length/2) - sin(beta)*(y-collision_rect_width/2);
        y3 = base_pose_y + sin(beta)*(x-collision_rect_length/2) + cos(beta)*(y-collision_rect_width/2);
        x4 = base_pose_x + cos(beta)*(x+collision_rect_length/2) - sin(beta)*(y-collision_rect_width/2);
        y4 = base_pose_y + sin(beta)*(x+collision_rect_length/2) + cos(beta)*(y-collision_rect_width/2);

        fx = base_pose_x + cos(beta)*(x)-sin(beta)*(y);
        fy = base_pose_y + sin(beta)*(x)+cos(beta)*(y);
        grid_map::Polygon polygon;
        polygon.setFrameId(elev_map.getFrameId());
        polygon.addVertex(Position( x1,  y1));
        polygon.addVertex(Position( x2,  y2));
        polygon.addVertex(Position( x3,  y3));
        polygon.addVertex(Position( x4,  y4));
        for (grid_map::PolygonIterator iterator(elev_map,polygon); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            if(elev_map.at("elevation_inpainted", *iterator) == elev_map.at("elevation_inpainted", *iterator)){
            if( elev_map.at("elevation_inpainted", *iterator) > collision_point_height) 
            {
                collision_points+=1;

            }}
            total_points+=1;

        }

        if((total_points-collision_points)*100/total_points < collision_free_threshold){
            for(int j=i;j<=sample_points;j++) collision_cost += (1/pow(2,j));
            if(return_totalcost) return collision_cost*100*collision_costFactor + global_costFactor*get_globalPlan_cost(fx,fy,sample_points);
            return collision_cost*100;
        }
    }
    if(return_totalcost) return collision_cost*100*collision_costFactor + global_costFactor*get_globalPlan_cost(fx,fy,sample_points);
    return collision_cost;
}


