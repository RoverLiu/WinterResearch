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

// ----------------------------------------------------------
// this class is designed to handle the data
class camera_pose
{
private:
    /* data */
    arm left;
    arm right;
    const double EPSILON = 0.0001;

    // ROS NodeHandle
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_priv;

    // std::string sub_topic_name = "~/human_pose";
    ros::Subscriber _human_pose_sub;

    // callback
    void HumanPoseCallback(const camera_msg::JointState::ConstPtr& msg);
    
    // funcs
    void filter_callback_dat(camera_msg::arm original, arm* to_save);

    void joint_copy(std::vector<float> src, joint* dest);

    bool is_same(double a, double b);

public:

    
    camera_pose(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    ~camera_pose();
};


#endif
