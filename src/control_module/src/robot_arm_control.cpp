/**
 * @file robot_arm_control.cpp
 * @author Rover
 * @brief Control robot arm with all basic functions
 * @version 0.2
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#include <string> // for string class
#include "robot_arm_control.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/Float64.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "arm.h"
/**
 * @brief Construct a new robot arm control::robot arm control object
 * 
 * @param nh Node handler for ros
 * @param nh_priv Previous node handler for ros
 * @param PLANNING_GROUP The name of the planning group in string
 * @param gripper_topic The topic name for the gripper control
 */
robot_arm_control::robot_arm_control(ros::NodeHandle nh, ros::NodeHandle nh_priv, const std::string PLANNING_GROUP, const std::string gripper_topic)  
:_nh(nh),_nh_priv(nh_priv), PLANNING_GROUP(PLANNING_GROUP), gripper_topic(gripper_topic)
{
    /*------------------------------------------set up moveit---------------------------------------*/
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    joint_model_group =  move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // We can get a list of all the groups in the robot:
    std::printf("Available Planning Groups:");
    std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
    
    // default
    move_group->setPlanningTime(30);
    move_group->setPlannerId("RRTConnectkConfigDefault");
    
    /*------------------------------------------set up kinematic state---------------------------------------*/
    robot_kinematic_state = move_group->getCurrentState();
    joint_names = joint_model_group->getVariableNames();
    std::cout<<"Again, the joint names:"<<std::endl;
    for (int i = 0; i < joint_names.size(); i++) {
        std::cout<< joint_names[i] <<std::endl;
    }

    /*------------------------------------------set up griper control---------------------------------------*/
    gripper_pub = nh.advertise<std_msgs::Float64>(gripper_topic, 1000);

    //------------------------------------------set up objects avoidance-------------------------------------
    // set up table and stand
    table.header.frame_id = move_group->getPlanningFrame();
    stand.header.frame_id = move_group->getPlanningFrame();
    table.id = "table";
    stand.id = "chocolate_stand";
    shape_msgs::SolidPrimitive primitive;
    shape_msgs::SolidPrimitive stand_primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.2;

    stand_primitive.type = stand_primitive.BOX;
    stand_primitive.dimensions.resize(3);
    stand_primitive.dimensions[0] = 0.05;
    stand_primitive.dimensions[1] = 1.0;
    stand_primitive.dimensions[2] = 0.15;

    // object position
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.1;

    geometry_msgs::Pose stand_pose;
    stand_pose.orientation.w = 1.0;
    stand_pose.position.x = 0.45;
    stand_pose.position.y = 0.0;
    stand_pose.position.z = 0.075;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(box_pose);
    table.operation = table.ADD;

    stand.primitives.push_back(stand_primitive);
    stand.primitive_poses.push_back(stand_pose);
    stand.operation = stand.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(table);
    collision_objects.push_back(stand);
    
    ROS_INFO( "Add objects into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
}

/**
 * @brief Destroy the robot arm control::robot arm control object
 * 
 */
robot_arm_control::~robot_arm_control() 
{
    delete move_group;
}

/**
 * @brief move to the given position
 * +x 向前移动
 * +y 像左侧桌子移动
 * @param goal The position to reach
 */
void robot_arm_control::auto_move_arm( geometry_msgs::Pose goal) 
{
    // print position to reach
    // std::cout<<goal.position<<std::endl;
    
    // set 
    move_group->setPoseTarget(goal);

    // plan
    move_group->plan(my_plan);

    // execuate
    move_group->move();
}

/**
 * @brief move arm with a straight line
 * 
 * @param waypoints a vector saves all changing angle to reach
 */
void robot_arm_control::CartesianPath_move_arm( std::vector<geometry_msgs::Pose> waypoints) 
{
    move_group->setStartStateToCurrentState();
    move_group->setMaxVelocityScalingFactor(0.1);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    // moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO("Trajectory Movement (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    move_group->execute(my_plan);
}

/**
 * @brief return w value for differetn direction
 * 
 * @param direction 1: front, 2: left, 3: right, 4 down, 5 up, 6 drop
 * @return geometry_msgs::Quaternion 
 */
geometry_msgs::Quaternion robot_arm_control::get_direction(int direction)
{
    geometry_msgs::Quaternion orientation;

    // get use tf to calculate orientation
    tf2::Quaternion myQuaternion;
    // tf2::Quaternion target_direction;

    switch(direction) {
        case 1  :
            myQuaternion.setRPY( 0, pi/2, 0 ); 
            break; //optional
        case 2  :
            myQuaternion.setRPY( -pi/2, pi/2, 0 ); 
            break; //optional
        case 3  :
            myQuaternion.setRPY( pi/2, pi/2, 0 ); 
            break; //optional

        case 4  :
            myQuaternion.setRPY( 0, pi, 0 ); 
            break; //optional

        case 5  :
            myQuaternion.setRPY( 0, 0, 0 ); 
            break; //optional
        
        case 6  :
            myQuaternion.setRPY( pi/2, 0, pi/2 ); 
            break; //optional


        // you can have any number of case statements.
        default : //Optional
            myQuaternion.setRPY( 0, pi/2, 0 ); 
    }

    myQuaternion.normalize();
    
    tf2::convert(myQuaternion, orientation);


    return orientation;
}

/**
 * @brief open/close gripper
 * 
 * @param state 0 means close, 1 means open
 */
void robot_arm_control::gripper_control( int state)
{
    // for now, the position of gripper is ranging between -20 to 20
    std_msgs::Float64 pos;
    if (state == 0) 
    {
        pos.data = 20;
        gripper_pub.publish(pos);
    }
    else
    {
        pos.data = -20;
        gripper_pub.publish(pos);
    }
    
}

/**
 * @brief get current position and orientation details
 * 
 * @return geometry_msgs::PoseStamped The current pos details
 */
geometry_msgs::PoseStamped robot_arm_control::get_current_pose() 
{
    geometry_msgs::PoseStamped current_pos;

    current_pos = move_group->getCurrentPose();

    // print out current pos
    // std::cout<<current_pos.pose.position<<std::endl;
    // std::cout<<current_pos.pose.orientation<<std::endl;

    // get angel detail
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // print the angle out
    // printf("Pos detail in angle:\n");
    // for (double i : joint_group_positions) 
    // {
    //     printf("%f\n", i);
    // }

    return current_pos;
}

/**
 * @brief reset the heading direction of gripper
 * 
 */
void robot_arm_control::reset_griper_direction() 
{
    // get current pose
    geometry_msgs::PoseStamped current_pose = get_current_pose();
    geometry_msgs::Pose target_pose = current_pose.pose;
    target_pose.orientation = get_direction(1);

    auto_move_arm(target_pose);
}

/**
 * @brief Reset arm to required posture with given angle
 * 
 * @param joint_group_positions THe angle degree for each joint
 */
void robot_arm_control::move_to_arm_pos_by_angle(std::vector<double> joint_group_positions) 
{
    move_group->setJointValueTarget(joint_group_positions);
    move_group->plan(my_plan);
    move_group->move();
}

/**
 * @brief move to the target position with given the id
 * 
 * @param id the index of angle in the joint_values
 */
void robot_arm_control::move_to_arm_pos_by_id(int id) 
{
    if (id < 0 || id > joint_values.size()) {
        // error
        ROS_INFO("Wrong ID is given");
        return;
    }
    move_group->setJointValueTarget(joint_values[id]);
    move_group->plan(my_plan);
    move_group->move();
}


/**
 * @brief detach the stand from the world
 * 
 */
void robot_arm_control::detach_stand_object()
{
    ROS_INFO("Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(stand.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
}

/**
 * @brief attach the stand from the world
 * 
 */
void robot_arm_control::attach_stand_object()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(stand);
    planning_scene_interface.addCollisionObjects(collision_objects);
}

/**
 * @brief calculate all possible joint angle groups 
 * for the given EEF pose
 * All potentail results are saved in joint_values
 * 
 * @param target 
 */
void robot_arm_control::calculate_joint_angles(geometry_msgs::Pose target) 
{
    joint_values.clear();
    auto bind_callback = boost::bind(& robot_arm_control::save_IK_callback, this, _1, _2, _3);
    robot_kinematic_state->setFromIK(joint_model_group, target, IK_timeout, bind_callback);

    std::cout<<joint_values.size()<<" solutions has been found under "<<IK_timeout<<std::endl;
}

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
bool robot_arm_control::save_IK_callback(robot_state::RobotState* robot_state, 
            const robot_state::JointModelGroup* joint_group, 
            const double* joint_group_variable_values)
{
    // const std::vector<std::string>& joint_names = joint_group->getVariableNames();

    for (std::size_t i = 0; i < joint_names.size(); i++)
    {
        // show
        // ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_group_variable_values[i]);
        
        // copy
        std::vector<double> dest;
        vector_copy(joint_group_variable_values, &dest, joint_names.size());
        joint_values.push_back(dest);
    }
    return false; // Depending on your desition scheme.
}

/**
 * @brief copy the vector from pointer to std::vector
 * 
 * @param src const double as source
 * @param dest target std::vector format (destination)
 * @param size the size of the pointer
 */
void robot_arm_control::vector_copy(const double* src, std::vector<double> *dest, int size) 
{
    for (std::size_t i = 0; i < size; ++i) {
        dest->push_back(src[i]);
    }
}

/**
 * @brief Get the angle positions for all solutions
 * loaded in the joint_values
 * 
 * @return std::vector<std::vector<std::vector<double>>> 
 * [[[1,1,1],[2,2,2],[3,3,3],[4,4,4],[5,5,5],...]]
 * n*7*3
 */
std::vector<std::vector<std::vector<double>>> robot_arm_control::get_angle_positions() 
{
    // loop through and save
    std::vector<std::vector<std::vector<double>>> res;
    std::cout<<"total "<< joint_values.size() << " solutions would be returned\n";
    for (int i = 0; i < joint_values.size(); i++) {
        res.push_back(get_angle_position(joint_values[i]));
    }
    return res;
}

/**
 * @brief Get the angle position object
 * 
 * @return std::vector<std::vector<double>> a group of vector for each joint 
 */
std::vector<std::vector<double>> robot_arm_control::get_angle_position(std::vector<double> angle)
{
    const std::vector< std::string > & link_names = move_group->getLinkNames();

    std::vector<std::vector<double>> res;
    
    // calculate
    robot_kinematic_state->setJointGroupPositions(joint_model_group, angle);

    // save data
    for (int i = 0; i < link_names.size(); i++) {
        // get the linker
        const Eigen::Isometry3d& link_state = 
            robot_kinematic_state->getGlobalLinkTransform(link_names[i]);
        // show
        // std::cout<<link_names[i]<<": \n" << link_state.translation() << "\n";
        // save
        std::vector<double> position;
        position.push_back(link_state.translation().x());
        position.push_back(link_state.translation().y());
        position.push_back(link_state.translation().z());
        res.push_back(position);
    }
    return res;
}

/**
 * @brief Get the orientation object
 * calculate the orientation of the hand by given arm detials
 * @param arm 
 * @param shoulder_to_hip_position the position transform from hip to shoulder
 * @return geometry_msgs::PoseStamped pose of arm
 */
geometry_msgs::PoseStamped robot_arm_control::get_pose_in_robot_frame(arm* arm, tf2::Vector3& shoulder_to_hip_position) 
{
    // get vector
    std::vector<double> orientation_vector;
    double x = 0;
    double y = 0;
    double z = 0;
    int count = 0;
    printf("[ROBOT ARM CONTROL] Hand index details: (%f, %f, %f, %d)\n", arm->hand_index.get_x(),
        arm->hand_index.get_y(), arm->hand_index.get_z(), arm->hand_index.is_reliable());
    if (arm->hand_index.is_reliable()) {
        x+=arm->hand_index.get_x();
        y+=arm->hand_index.get_y();
        z+=arm->hand_index.get_z();
        count += 1;
    }
    if (arm->hand_pinky.is_reliable()) {
        x+=arm->hand_pinky.get_x();
        y+=arm->hand_pinky.get_y();
        z+=arm->hand_pinky.get_z();
        count += 1;
    }
    if (arm->hand_thumb.is_reliable()) {
        x+=arm->hand_thumb.get_x();
        y+=arm->hand_thumb.get_y();
        z+=arm->hand_thumb.get_z();
        count += 1;
    }
    // printf("[ROBOT ARM CONTROL] Orientation in the middle: (%f, %f, %f)\n", x,y,z);

    // get vector relative to wrist
    if (count != 0) {
        x = x/count - arm->wrist.get_x();
        y = y/count - arm->wrist.get_y();
        z = z/count - arm->wrist.get_z();
    }
    else {
        z = 1;
    }

    // convert to pose
    geometry_msgs::PoseStamped camera_frame;
    geometry_msgs::PoseStamped robot_frame;

    // use the pose relative to hip with length considered
    // note: different arm length matters
    // size_factor*(human wrist relative to human shoulder)+(robot shoulder relative to robot hip)
    double size_factor = ROBOT_ARM_LENGTH / arm->get_arm_length();

    printf("[ROBOT ARM CONTROL] wrist postion in camera frame: (%f, %f, %f)\n\t shoulder postion in camera frame: (%f, %f, %f)\n",
        arm->wrist.get_x(), arm->wrist.get_y(), arm->wrist.get_z(),
        arm->shoulder.get_x(), arm->shoulder.get_y(), arm->shoulder.get_z());

    // get position
    camera_frame.pose.position.x = size_factor*(arm->wrist.get_x() - arm->shoulder.get_x());
    camera_frame.pose.position.y = size_factor*(arm->wrist.get_y() - arm->shoulder.get_y());
    camera_frame.pose.position.z = size_factor*(arm->wrist.get_z() - arm->shoulder.get_z());

    // set orientation
    printf("[ROBOT ARM CONTROL] Orientation: (%f, %f, %f)\n", x,y,z);
    // tf2::Vector3 axis;
    // axis.setX(x);
    // axis.setY(y);
    // axis.setZ(z);
    // // note: the orientaion of hand could be improved by giving a rotation
    // tf2::Quaternion myQuaternion(axis, 0.0);
    // myQuaternion = myQuaternion.normalize();
    // camera_frame.pose.orientation.x = myQuaternion.getX();
    // camera_frame.pose.orientation.y = myQuaternion.getY();
    // camera_frame.pose.orientation.z = myQuaternion.getZ();
    // camera_frame.pose.orientation.w = myQuaternion.getW();
    std::vector<double> original, goal;
    original.push_back(0);
    original.push_back(0);
    original.push_back(1);

    goal.push_back(x);
    goal.push_back(y);
    goal.push_back(z);
    
    std::vector<double> cross_product = CrossProduct1D(original, goal);

    camera_frame.pose.orientation.x = cross_product[0];
    camera_frame.pose.orientation.y = cross_product[1];
    camera_frame.pose.orientation.z = cross_product[2];
    camera_frame.pose.orientation.w = std::sqrt(x*x+y*y+z*z)+z;

    // rotate to robot frame
    rotate_from_camera_to_robot(camera_frame, robot_frame);

    robot_frame.pose.position.x += shoulder_to_hip_position.getX();
    robot_frame.pose.position.y += shoulder_to_hip_position.getY();
    robot_frame.pose.position.z += shoulder_to_hip_position.getZ();

    printf("[ROBOT ARM CONTROL] camera frame: (%f, %f, %f)\n \trobot frame: (%f, %f, %f)\n \tsize factor: %f\n",
        camera_frame.pose.position.x, camera_frame.pose.position.y, camera_frame.pose.position.z,
        robot_frame.pose.position.x, robot_frame.pose.position.y, robot_frame.pose.position.z,
        size_factor);

    return robot_frame;
}

std::vector<double> robot_arm_control::CrossProduct1D(std::vector<double> const &a, std::vector<double> const &b)
{
    std::vector<double> r (a.size());  
    r[0] = a[1]*b[2]-a[2]*b[1];
    r[1] = a[2]*b[0]-a[0]*b[2];
    r[2] = a[0]*b[1]-a[1]*b[0];
    return r;
}

/**
 * @brief transform the pose in camera frame to robot frame
 * 
 * @param in camera frame
 * NOTE: z would be inverse (human frame and camera frame)
 * @param out   robot frame
 */
void robot_arm_control::rotate_from_camera_to_robot(geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out)
{
    // // align the frame
    // in.pose.position.z *= -1;
    // generate transform
    geometry_msgs::TransformStamped Transform;
    Transform.child_frame_id = move_group->getPlanningFrame();
    
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(pi/2,0,pi/2);
    myQuaternion = myQuaternion.normalize();
    Transform.transform.rotation.x = myQuaternion.getX();
    Transform.transform.rotation.y = myQuaternion.getY();
    Transform.transform.rotation.z = myQuaternion.getZ();
    Transform.transform.rotation.w = myQuaternion.getW();

    // do transform
    tf2::doTransform(in, out, Transform);
}