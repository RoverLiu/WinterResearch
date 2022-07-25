/**
 * @file robot_arm_control.h
 * @author Rover
 * @brief Control robot arm with all basic functions
 * @version 0.2
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ROBOT_ARM_CONTROL_H
#define __ROBOT_ARM_CONTROL_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Vector3.h>
// #include <trajectory_msgs.h>
#include <iostream>
#include <string> // for string class

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include "arm.h"

class robot_arm_control 
{
    public:
        // methods
        robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv,  const std::string PLANNING_GROUP, const std::string gripper_topic);
        ~robot_arm_control();

        /**
         * @brief move arm to required position
         * 
         * @param goal the position to ge
         */
        void auto_move_arm( geometry_msgs::Pose goal);

        /**
         * @brief move arm with a straight line
         * 
         * @param waypoints a vector saves all changing angle to reach
         */
        void CartesianPath_move_arm( std::vector<geometry_msgs::Pose> waypoints);

        /**
         * @brief Get the direction object
         * get orientation value for required direction 
         * @param direction (1: front, 2: left, 3: right)
         * @return geometry_msgs::Quaternion The orientation in desired form
         */
        geometry_msgs::Quaternion get_direction(int direction);

        /**
         * @brief open or close gripper 
         * 
         * @param state (0 means close, 1 means open)
         */
        void gripper_control( int state);

        /**
         * @brief Get the current pose object
         * 
         * @return geometry_msgs::PoseStamped 
         */
        geometry_msgs::PoseStamped get_current_pose();

        /**
         * @brief reset the direction of the gripper
         * 
         */
        void reset_griper_direction();

        /**
         * @brief Reset the position of the arm to desired angle
         * 
         * @param joint_group_positions The angle for each joint
         */
        void move_to_arm_pos_by_angle(std::vector<double> joint_group_positions);
        
        /**
         * @brief move to the target position with given the id
         * 
         * @param id the index of angle in the joint_values
         */
        void move_to_arm_pos_by_id(int id);

        /**
         * @brief detach the stand from the world
         * 
         */
        void detach_stand_object();

        /**
         * @brief attach the stand from the world
         * 
         */
        void attach_stand_object();

        /**
         * @brief calculate all possible joint angle groups 
         * for the given EEF pose
         * All potentail results are saved in joint_values
         * 
         * @param target 
         */
        void calculate_joint_angles(geometry_msgs::Pose target);

        /**
         * @brief Get the angle positions for all solutions
         * loaded in the joint_values
         * 
         * @return std::vector<std::vector<std::vector<double>>> 
         * [[[1,1,1],[2,2,2],[3,3,3],[4,4,4],[5,5,5],...]]
         * n*7*3
         */
        std::vector<std::vector<std::vector<double>>> get_angle_positions();

        /**
         * @brief Get the angle position object
         * 
         * @return std::vector<std::vector<double>> a group of vector for each joint 
         */
        std::vector<std::vector<double>> get_angle_position(std::vector<double> angle);
          
        /**
         * @brief Get the pose object
         * calculate the pose of the hand by given arm detials
         * @param arm 
         * @param shoulder_to_hip_position the position transform from hip to shoulder
         * @return geometry_msgs::PoseStamped pose of arm
         */
        geometry_msgs::PoseStamped get_pose_in_robot_frame(arm* arm, tf2::Vector3& shoulder_to_hip_position);

        /**
         * @brief transform the pose in camera frame to robot frame
         * 
         * @param in
         * @param out
         */
        void rotate_from_camera_to_robot(geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out);
       
    private:
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        // gripper control pub
        ros::Publisher gripper_pub;

        // choose planning group for the arm
        const std::string PLANNING_GROUP;

        // this is designed for yumi only
        // control gripper
        const std::string gripper_topic;

        // The :move_group_interface:`MoveGroupInterface` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface *move_group;

        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Raw pointers are frequently used to refer to the planning group for improved performance.
        const robot_state::JointModelGroup* joint_model_group;
        robot_state::RobotStatePtr robot_kinematic_state;
        // const robot_state::JointModelGroup* right_joint_model_group;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // a vector to save all results (angles)
        std::vector<std::vector<double>> joint_values;

        // a vector saves all joint names
        std::vector<std::string> joint_names;

        const double pi = 3.1415926535;
        const double IK_timeout = 0.1;
        const double ROBOT_ARM_LENGTH = 0.76;

        // collision object
        moveit_msgs::CollisionObject table;
        moveit_msgs::CollisionObject stand;

        /**
         * @brief callback function for IK solver
         * will always return false, which pushes the 
         * solver to find all possible solutions with target pose
         * 
         * @param robot_state 
         * @param joint_group 
         * @param joint_group_variable_values 
         * @return true 
         * @return false 
         */
        bool save_IK_callback(robot_state::RobotState* robot_state, 
                const robot_state::JointModelGroup* joint_group, 
                const double* joint_group_variable_values);
        
        /**
         * @brief copy the vector from pointer to std::vector
         * 
         * @param src const double as source
         * @param dest target std::vector format (destination)
         * @param size the size of the pointer
         */
        void vector_copy(const double* src, std::vector<double> *dest, int size);

        /**
         * @brief cross product
         */
        std::vector<double> CrossProduct1D(std::vector<double> const &a, std::vector<double> const &b);
     
};

#endif