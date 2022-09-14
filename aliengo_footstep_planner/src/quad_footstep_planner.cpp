#include "aliengo_footstep_planner/quad_footstep_planner.h"

bool collision;

void plan_footsteps(){
    // cout<<joystick_vals[0]<<"  "<<joystick_vals[1]<<endl;
}

void collision_check(ros::Publisher poly_pub){
    int sample_points = (int)(horizon_length*max_steplength/grid_map_resolution);
    double v = max(0.0,(joystick_vals[1]*max_forward_vel));
    double w = (joystick_vals[0]*max_angular_vel);
    double time_period = (max_steplength*horizon_length)/max_forward_vel;
    int total_points = 1;
    int collision_points = 0;
    for (int i=0;i<=sample_points;i++){
        double t = i*time_period/sample_points;
        double x1,x2,x3,x4,y1,y2,y3,y4,x,y;
        if(w != 0){
            x = (v/w)*sin(w*t);
            y = -(v/w)*(cos(w*t)-1);
        }
        else{
            x = v*t;
            y = 0;
        }
        double beta = w*t + base_pose_yaw; 
        x1 = elev_map.getPosition()[0] + x + cos(beta)*(collision_rect_length/2) - sin(beta)*(collision_rect_width/2);
        y1 = elev_map.getPosition()[1] + y + sin(beta)*(collision_rect_length/2) + cos(beta)*(collision_rect_width/2);
        x2 = elev_map.getPosition()[0] + x + cos(beta)*(-collision_rect_length/2) - sin(beta)*(collision_rect_width/2);
        y2 = elev_map.getPosition()[1] + y + sin(beta)*(-collision_rect_length/2) + cos(beta)*(collision_rect_width/2);
        x3 = elev_map.getPosition()[0] + x + cos(beta)*(-collision_rect_length/2) - sin(beta)*(-collision_rect_width/2);
        y3 = elev_map.getPosition()[1] + y + sin(beta)*(-collision_rect_length/2) + cos(beta)*(-collision_rect_width/2);
        x4 = elev_map.getPosition()[0] + x + cos(beta)*(collision_rect_length/2) - sin(beta)*(-collision_rect_width/2);
        y4 = elev_map.getPosition()[1] + y + sin(beta)*(collision_rect_length/2) + cos(beta)*(-collision_rect_width/2);
        grid_map::Polygon polygon;
        polygon.setFrameId(elev_map.getFrameId());
        polygon.addVertex(Position( x1,  y1));
        polygon.addVertex(Position( x2,  y2));
        polygon.addVertex(Position( x3,  y3));
        polygon.addVertex(Position( x4,  y4));

        geometry_msgs::PolygonStamped message;
        grid_map::PolygonRosConverter::toMessage(polygon, message);
        poly_pub.publish(message);

        for (grid_map::PolygonIterator iterator(elev_map,polygon); !iterator.isPastEnd(); ++iterator) {
            const Index index(*iterator);
            if(elev_map.at("elevation", *iterator) == elev_map.at("elevation", *iterator)){
            if( elev_map.at("elevation", *iterator) > collision_point_height) 
            {
                collision_points+=1;

            }}
            total_points+=1;

        }

    }
    if (((total_points-collision_points)/total_points)*100 > collision_threshold) collision = false;
    else collision = true;
    //cout<<collision<<endl;

}