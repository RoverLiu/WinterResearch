#include "camera_pose.h"
#include <ros/ros.h>

camera_pose::camera_pose(ros::NodeHandle nh, ros::NodeHandle nh_priv)
:_nh(nh),_nh_priv(nh_priv)
{
    _human_pose_sub = nh.subscribe("/human_pose", 1000, & camera_pose::HumanPoseCallback, this);
}

camera_pose::~camera_pose()
{
}

void camera_pose::HumanPoseCallback(const camera_msg::JointState::ConstPtr& msg) 
{
    filter_callback_dat(msg->left, &left);
    filter_callback_dat(msg->right, &right);
}

/**
 * @brief filter and save the data from rostopic
 * 
 * @param original 
 * @param to_save 
 */
void camera_pose::filter_callback_dat(camera_msg::arm original, arm* to_save)
{
    joint_copy(original.elbow, &(to_save->elbow));
    joint_copy(original.hand_index, &(to_save->hand_index));
    joint_copy(original.hand_pinky, &(to_save->hand_pinky));
    joint_copy(original.hand_thumb, &(to_save->hand_thumb));
    joint_copy(original.hip, &(to_save->hip));
    joint_copy(original.shoulder, &(to_save->shoulder));
    joint_copy(original.wrist, &(to_save->wrist));
    
    // show results
    printf("[Arm Log] elbow - x: %f, y: %f, z: %f\n", to_save->elbow.get_x(), to_save->elbow.get_y(), to_save->elbow.get_z());
}

/**
 * @brief copy the joint data
 * 
 * @param src 
 * @param dest 
 */
void camera_pose::joint_copy(std::vector<float> src, joint* dest)
{
    // invalid
    if (is_same(src[0], 0.0) 
        && is_same(src[1], 0.0)
        && is_same(src[2], -1.0)) 
    {
        dest->set_reliable(false);
    }
    else {
        // valid
        dest->set_x(src[0]);
        dest->set_y(src[1]);
        dest->set_z(src[2]);
        dest->set_reliable(true);
    }
}

// compare two double number
bool camera_pose::is_same(double a, double b)
{
    return fabs(a - b) < EPSILON;
}

