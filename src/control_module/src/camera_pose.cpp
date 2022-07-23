#include "camera_pose.h"
#include <ros/ros.h>

camera_pose::camera_pose(ros::NodeHandle nh, ros::NodeHandle nh_priv)
:_nh(nh),_nh_priv(nh_priv)
{
    _human_pose_sub = nh.subscribe("/human_pose", 1000, & camera_pose::HumanPoseCallback, this);
    is_left_arm_ready = false;
    is_right_arm_ready = false;

    left_ready = new arm();
    right_ready = new arm();
}

camera_pose::~camera_pose()
{
    free(left_ready);
    free(right_ready);
}

void camera_pose::HumanPoseCallback(const camera_msg::JointState::ConstPtr& msg) 
{
    is_left_arm_ready = filter_callback_dat(msg->left, &left);
    if (is_left_arm_ready) {
        arm::copy(&left, left_ready);
    }

    is_right_arm_ready = filter_callback_dat(msg->right, &right);
    if (is_right_arm_ready) {
        arm::copy(&right, right_ready);
    }
}

/**
 * @brief filter and save the data from rostopic
 * 
 * @param original 
 * @param to_save 
 */
bool camera_pose::filter_callback_dat(camera_msg::arm original, arm* to_save)
{
    joint::joint_copy(original.elbow, &(to_save->elbow));
    joint::joint_copy(original.hand_index, &(to_save->hand_index));
    joint::joint_copy(original.hand_pinky, &(to_save->hand_pinky));
    joint::joint_copy(original.hand_thumb, &(to_save->hand_thumb));
    joint::joint_copy(original.hip, &(to_save->hip));
    joint::joint_copy(original.shoulder, &(to_save->shoulder));
    joint::joint_copy(original.wrist, &(to_save->wrist));
    
    // show results
    printf("[Arm Log] elbow - x: %f, y: %f, z: %f\n", to_save->elbow.get_x(), to_save->elbow.get_y(), to_save->elbow.get_z());

    // check state
    bool state = true;
    state = state && to_save->elbow.is_reliable();
    state = state && to_save->shoulder.is_reliable();
    state = state && to_save->wrist.is_reliable();
    state = state && to_save->hip.is_reliable();
    return state;
}

/**
 * @brief Get the left arm object
 * return null when the data is not updated or ready
 * 
 * @return arm *
 */
arm* camera_pose::get_left_arm() 
{
    if (is_left_arm_ready) {
        // ready
        return left_ready;
    }
    else {
        return NULL;
    }
    is_left_arm_ready = false;
}

/**
 * @brief Get the left arm object
 * return null when the data is not updated or ready
 * 
 * @return arm *
 */
arm* camera_pose::get_right_arm() 
{
    if (is_right_arm_ready) {
        // ready
        return right_ready;
    }
    else {
        return NULL;
    }
    is_right_arm_ready = false;
}

