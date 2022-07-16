#include "camera_pose.h"
#include <ros/ros.h>

int main( int argc, char** argv )
{
    // Initialize the ros
    ros::init( argc, argv, "Control_Module");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );
    camera_pose my_camera_manager(nh, nh_priv);

    while( ros::ok() )
    { 
        // //Simulate robot's battery
        // my_camera.battery_simulation();

        // // robots start deliver pizzas
        // my_robot.deliver_pizza();
        ROS_DEBUG("Normal Running");

        // ros::spinOnce();
        ros::Duration(1.0).sleep(); // sleep for one second
    }
    
    return 0;
}
