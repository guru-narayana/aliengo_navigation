#include "quad_footstep_planner.h"
#include <ros/ros.h>



string grid_map_topic;
double grid_map_resolution;
double max_steplength;
double min_steplength;

void get_params(ros::NodeHandle& nh){
    nh.param("Grid_map_topic", grid_map_topic, std::string("/elevation_map"));
    nh.param("Grid_map_resolution", grid_map_resolution, 5.0);
    nh.param("max_steplength", max_steplength, 3*grid_map_resolution);
    nh.param("min_steplength", min_steplength, grid_map_resolution);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "Quad_footstep_planner_node");
    ros::NodeHandle nh("Quad_footstep_planner");

    get_params(nh);

    cout<<grid_map_topic<<endl;
    cout<<grid_map_resolution<<endl;
    cout<<max_steplength<<endl;
    cout<<min_steplength<<endl;

    ros::spin();
    
    return EXIT_SUCCESS;
}
