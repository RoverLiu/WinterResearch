/**
 * @file camera_pose.h
 * @author Rover (you@domain.com)
 * @brief This module handles the data for the human pose
 * It manages the data from camera module 
 * @version 0.1
 * @date 2022-07-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CAMERA_POSE_H
#define __CAMERA_POSE_H

#include "arm.h"
#include <ros/ros.h>
#include "camera_msg/JointState.h"
#include <vector>
// ----------------------------------------------------------
// this class is designed to handle the data
class camera_pose
{
private:
    /* data */
    arm left;
    arm right;
    arm* left_ready;
    arm* right_ready;

    // ROS NodeHandle
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_priv;

    // std::string sub_topic_name = "~/human_pose";
    ros::Subscriber _human_pose_sub;

    // arm data state
    bool is_left_arm_ready;
    bool is_right_arm_ready;

    // callback
    void HumanPoseCallback(const camera_msg::JointState::ConstPtr& msg);
    
    // funcs
    /**
     * @brief filter and save the data from rostopic
     * 
     * @param original 
     * @param to_save 
     * @return return whether all joint required is valid
     */
    bool filter_callback_dat(camera_msg::arm original, arm* to_save);






public:
    // constructor
    camera_pose(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    ~camera_pose();

    /**
     * @brief Get the left arm object
     * return null when the data is not updated or ready
     * 
     * @return arm *
     */
    arm* get_left_arm();

    /**
     * @brief Get the left arm object
     * return null when the data is not updated or ready
     * 
     * @return arm *
     */
    arm* get_right_arm();

};


#endif
