#include "camera_pose.h"
#include <ros/ros.h>
#include "arm_manager.h"
#include "control_manager.h"

int main( int argc, char** argv )
{
    // Initialize the ros
    ros::init( argc, argv, "Control_Module");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );
    ros::AsyncSpinner spinner(2);
    spinner.start();

    control_manager my_control_manager(nh, nh_priv);

    while( ros::ok() )
    { 
        // //Simulate robot's battery
        // my_camera.battery_simulation();

        // // robots start deliver pizzas
        // my_robot.deliver_pizza();
        ROS_DEBUG("Normal Running");
        std::cout<<"---------------------------Looping----------------------\n";

        my_control_manager.imitation();

        ros::spinOnce();
        ros::Duration(1.0).sleep(); // sleep for one second
    }
    
    return 0;
}
