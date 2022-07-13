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

// ----------------------------------------------------------
// this class is designed to handle the data
class camera_pose
{
private:
    /* data */
    arm left;
    arm right;

    // ROS NodeHandle
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_priv;
    
public:
    camera_pose(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    ~camera_pose();
};


#endif
