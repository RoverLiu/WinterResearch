/**
 * @file arm_manager.cpp
 * @author Rover
 * @brief The manager handles all process for picking up the chocolate
 * Taking order details come from UI and processing orders by camera and robot arm
 * The chocolate will be delivered to the specific positions for client. 
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "arm_manager.h"
#include <iostream>
#include <string> // for string class
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
// #include "order_handler.h"
// #include "camera_handler.h"
// #include "play_audio.h"

/**
 * @brief Construct a new arm manager::arm manager object
 * 
 * @param nh Node handler for ros
 * @param nh_priv Previous node handler for ros
 */
arm_manager::arm_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv) 
:_nh(nh),_nh_priv(nh_priv)
{

    left_arm = new robot_arm_control(nh, nh_priv, LEFT_PLANNING_GROUP, LEFT_GRIPPER_TOPIC);
    right_arm = new robot_arm_control(nh, nh_priv, RIGHT_PLANNING_GROUP, RIGHT_GRIPPER_TOPIC);
    // my_camera = new camera_handler(nh, nh_priv);
    // my_orders = new order_handler(nh, nh_priv);

    // set default position
    // roughly +- 0.6 is the maximum distance it cango
    // default_start_right_pos.position.x = default_start_right_pos_x;
    // default_start_right_pos.position.y = -default_start_right_pos_y;
    // default_start_right_pos.position.z = default_start_right_pos_z;
    // default_start_right_pos.orientation = left_arm->get_direction(1);

    // default_start_left_pos.position.x = default_start_right_pos_x;
    // default_start_left_pos.position.y = default_start_right_pos_y;
    // default_start_left_pos.position.z = default_start_right_pos_z;
    // default_start_left_pos.orientation = right_arm->get_direction(1);

    // reset
    left_arm->move_to_arm_pos_by_angle(left_arm_default_angle);
    right_arm->move_to_arm_pos_by_angle(right_arm_default_angle);

    // // get current pos
    // left_arm->get_current_pose();
    // right_arm->get_current_pose();
    
    // left_arm->auto_move_arm(default_start_left_pos);
    // right_arm->auto_move_arm(default_start_right_pos);

    // // move to default position 
    left_arm->move_to_arm_pos_by_angle(left_arm_finish_angle);
    right_arm->move_to_arm_pos_by_angle(right_arm_finish_angle);
    left_arm->gripper_control(1);
    right_arm->gripper_control(1);

    // move to default position 
    std::cout<<"right pos default angle"<<std::endl;
    right_arm->get_current_pose();

    std::cout<<"left pos default angle"<<std::endl;
    left_arm->get_current_pose();
 
    std::cout<<"-------------------reset finished--------------------"<<std::endl;

    // set up shoulder postion
    shoulder_position_left.setX(0.05355);
    shoulder_position_left.setY(0.07250);
    shoulder_position_left.setZ(0.41492);

    shoulder_position_right.setX(0.05355);
    shoulder_position_right.setY(-0.07250);
    shoulder_position_right.setZ(0.41492);

}

/**
 * @brief Destroy the arm manager::arm manager object
 * 
 */
arm_manager::~arm_manager()
{
    delete left_arm;
    delete right_arm;
}


/**
 * @brief wait until a input from keyboard
 * 
 */
void arm_manager::wait() 
{
    std::cout<<"Wait to continue, enter anything"<<std::endl;
    std::string a;
    std::cin >> a;
}

/**
 * @brief Get the left shoulder position object
 * 
 * @return tf2::Vector3 
 */
tf2::Vector3 arm_manager::get_left_shoulder_position() 
{
    return shoulder_position_left;
}

/**
 * @brief Get the right shoulder position object
 * 
 * @return tf2::Vector3 
 */
tf2::Vector3 arm_manager::get_right_shoulder_position()
{
    return shoulder_position_right;
}

/**
 * @brief Get the left arm object
 * 
 * @return robot_arm_control* 
 */
robot_arm_control* arm_manager::get_left_arm()
{
    return left_arm;
}

/**
 * @brief Get the right arm object
 * 
 * @return robot_arm_control* 
 */
robot_arm_control* arm_manager::get_right_arm()
{
    return right_arm;
}