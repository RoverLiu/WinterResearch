/**
 * @file arm_manager.h
 * @author Rover 
 * @brief The manager handles all process for imitation
 * Taking order details come from UI and processing orders by camera and robot arm
 * The chocolate will be delivered to the specific positions for client. 
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ARM_MANAGER_H
#define __ARM_MANAGER_H
#include <iostream>
#include <string> // for string class
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <tf2/LinearMath/Vector3.h>

class arm_manager
{
    private:
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        // arms to control
        robot_arm_control * left_arm;
        robot_arm_control * right_arm;

        // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
        // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
        // are used interchangably.
        const std::string LEFT_PLANNING_GROUP = "left_arm";
        const std::string RIGHT_PLANNING_GROUP = "right_arm";

        const std::string LEFT_GRIPPER_TOPIC = "/yumi/gripper_l_effort_cmd";
        const std::string RIGHT_GRIPPER_TOPIC = "/yumi/gripper_r_effort_cmd";

        // these angles are set for robot arm itself
        const std::vector<double> right_arm_default_angle = {-0.000103,
            -2.268631,
            -2.356222,
            0.523748,
            -0.000088,
            0.702391,
            -0.000187
        };

        const std::vector<double> left_arm_default_angle = {0.000144,
            -2.268888,
            2.356080,
            0.523852,
            0.000080,
            0.698850,
            -0.000160
        };

        // my defined position (beginning or ending position for each task)
        const std::vector<double> right_arm_finish_angle = {0.442606,
            -2.394394,
            -1.717076,
            0.384959,
            0.315967,
            0.872647,
            0.135105
        };

        const std::vector<double> left_arm_finish_angle = {-0.463526,
            -2.399707,
            1.691992,
            0.379761,
            -0.326887,
            0.879700,
            -0.141335
        };

        // default position for robot arm to start
        // geometry_msgs::Pose default_start_right_pos;
        // geometry_msgs::Pose default_start_left_pos;
        // // default start position value (left arm using the opposite y value)
        // float default_start_right_pos_x = 0.1;
        // float default_start_right_pos_y = 0.3;
        // float default_start_right_pos_z = 0.25;

        // float default_drop_pos_x = 0.50;
        // float default_drop_pos_y = 0.00;
        // float default_drop_pos_z = 0.25;
        
        // data from xacro configuration file
        tf2::Vector3 shoulder_position_left;
        tf2::Vector3 shoulder_position_right;

    public:
        // methods
        arm_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~arm_manager();

        // testing methods
        void wait();

        /**
         * @brief Get the left shoulder position object
         * 
         * @return tf2::Vector3 
         */
        tf2::Vector3 get_left_shoulder_position();

        /**
         * @brief Get the right shoulder position object
         * 
         * @return tf2::Vector3 
         */
        tf2::Vector3 get_right_shoulder_position();

        /**
         * @brief Get the left arm object
         * 
         * @return robot_arm_control* 
         */
        robot_arm_control* get_left_arm();

        /**
         * @brief Get the right arm object
         * 
         * @return robot_arm_control* 
         */
        robot_arm_control* get_right_arm();

        

};
#endif