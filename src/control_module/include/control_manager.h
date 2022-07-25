/**
 * @file control_manager.h
 * @author Rover (you@domain.com)
 * @brief This class controls camera data, handles robot pose
 * and monitor the state
 * @version 0.1
 * @date 2022-07-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __CONTROL_MANAGER_H
#define __CONTROL_MANAGER_H
#include <ros/ros.h>

#include "arm_manager.h"
#include "camera_pose.h"
#include "difference_calculator.h"
#include <geometry_msgs/Quaternion.h>

class control_manager
{
private:
    /* data */
    // ROS NodeHandle
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_priv;

    camera_pose* my_camera_manager;

    arm_manager* my_arm_manager;

    difference_calculator* my_calculator;

    const double pi = 3.1415926;
    double MINIMUM_Z = 0.12;

public:
    control_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    ~control_manager();

    /**
     * @brief imitation once for both arm
     * 
     */
    void imitation();

    /**
     * @brief imitate based on given arm 
     * 
     * @param camera_arm the human arm detail
     * @param robot_arm the robot arm detail
     * @param shoulder the shoulder position in robot frame
     */
    void imitate_one_arm(arm* camera_arm, robot_arm_control* robot_arm, tf2::Vector3 shoulder);
    
    /**
     * @brief convert the human arm in camera frame to the robot frame
     * Make sure the x,y,z axis are aligned
     * @param human_positions [wrist/elbow/shoulder][x,y,z]
     * @param positions_in_robot_frame output in [wrist/elbow/shoulder][x,y,z]
     * @param arm_controller calculator (have detials for transform)
     * 
     */
    void convert_human_arm_to_robot_frame(std::vector<std::vector<double>>& human_positions,
        std::vector<std::vector<double>>& positions_in_robot_frame, robot_arm_control* arm_controller);

    /**
     * @brief convert the point in camera frame to the robot frame
     * Make sure the x,y,z axis are aligned
     * @param position [x,y,z]
     * @param point_robot_frame output in [x,y,z] vector
     * @param arm_controller calculator (have detials for transform)
     * 
     */
    void convert_point_to_robot_frame(std::vector<double>& position,
        std::vector<double>& point_robot_frame, robot_arm_control* arm_controller);
};





#endif