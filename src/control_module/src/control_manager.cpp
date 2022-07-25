#include "control_manager.h"
#include "arm.h"
#include <tf/tf.h>

control_manager::control_manager(ros::NodeHandle nh, ros::NodeHandle nh_priv)
:_nh(nh),_nh_priv(nh_priv)
{
    my_camera_manager = new camera_pose(nh, nh_priv);
    my_arm_manager = new arm_manager(nh, nh_priv);
    my_calculator = new difference_calculator();
}

control_manager::~control_manager()
{
    free(my_arm_manager);
    free(my_camera_manager);
    free(my_calculator);
}

/**
 * @brief imitation once for both arm
 * 
 */
void control_manager::imitation() 
{
    std::cout<<"--------start imitation------------\n";
    // imitate the left arm
    arm* camera = my_camera_manager->get_left_arm();
    robot_arm_control* robot = my_arm_manager->get_left_arm();
    tf2::Vector3 shoulder = my_arm_manager->get_left_shoulder_position();
    imitate_one_arm(camera, robot, shoulder);

    // imitate the right arm
    camera = my_camera_manager->get_right_arm();
    robot = my_arm_manager->get_right_arm();
    shoulder = my_arm_manager->get_right_shoulder_position();
    imitate_one_arm(camera, robot, shoulder);
    std::cout<<"--------finish imitation------------\n";

}

void control_manager::imitate_one_arm(arm* camera_arm, robot_arm_control* robot_arm, tf2::Vector3 shoulder) 
{
    if (camera_arm != NULL) {
        // get end pose in robot frame from camera 
        geometry_msgs::PoseStamped EEF_pose = 
            robot_arm->get_pose_in_robot_frame(camera_arm, shoulder);

        // protect the gripper
        if (EEF_pose.pose.position.z < MINIMUM_Z) {
            EEF_pose.pose.position.z = MINIMUM_Z;
        }
        
        printf("CONTROL MANAGER: Robot target position: %f %f %f\n", EEF_pose.pose.position.x,
                EEF_pose.pose.position.y,EEF_pose.pose.position.z);

        // let the moveit calculate the potential solutions
        robot_arm->calculate_joint_angles(EEF_pose.pose);
        std::vector<std::vector<std::vector<double>>> positions =
            robot_arm->get_angle_positions();
        
        // no valid position found
        if (positions.size() == 0) {
            std::cout<<"LOGGER: No valid positon found, skip\n";
            return;
        }

        // find the best solution by comparison
        std::vector<std::vector<double>> positions_camera_frame;
        std::vector<std::vector<double>> positions_robot_frame;
        positions_camera_frame = camera_arm->get_arm_positions();
        convert_human_arm_to_robot_frame(positions_camera_frame,
            positions_robot_frame, robot_arm);

        int index = my_calculator->get_best_index(positions, 
                                    positions_robot_frame);

        // exectue the command    
        robot_arm->move_to_arm_pos_by_id(index);
    }
    else {
        std::cout<<"ERROR: invalid camera pose received\n";
    }
} 

/**
 * @brief convert the human arm in camera frame to the robot frame
 * Make sure the x,y,z axis are aligned
 * @param human_positions [wrist/elbow/shoulder][x,y,z]
 * @param positions_in_robot_frame output in [wrist/elbow/shoulder][x,y,z]
 * @param arm_controller calculator (have detials for transform)
 * 
 */
void control_manager::convert_human_arm_to_robot_frame(std::vector<std::vector<double>>& human_positions,
    std::vector<std::vector<double>>& positions_in_robot_frame, robot_arm_control* arm_controller)
{
    // caclculate
    std::vector<double> wrist_out;
    std::vector<double> elbow_out;
    std::vector<double> shoulder_out;

    convert_point_to_robot_frame(human_positions[0], shoulder_out, arm_controller);
    convert_point_to_robot_frame(human_positions[1], elbow_out, arm_controller);
    convert_point_to_robot_frame(human_positions[2], wrist_out, arm_controller);
    
    // extract the data
    positions_in_robot_frame.push_back(shoulder_out);
    positions_in_robot_frame.push_back(elbow_out);
    positions_in_robot_frame.push_back(wrist_out);
}

/**
 * @brief convert the point in camera frame to the robot frame
 * Make sure the x,y,z axis are aligned
 * @param position [x,y,z]
 * @param point_robot_frame output in [x,y,z] vector
 * @param arm_controller calculator (have detials for transform)
 * 
 */
void control_manager::convert_point_to_robot_frame(std::vector<double>& position,
std::vector<double>& point_robot_frame, robot_arm_control* arm_controller)
{
    // convert format 
    geometry_msgs::PoseStamped in;
    geometry_msgs::PoseStamped out;

    in.pose.position.x = position[0];
    in.pose.position.y = position[1];
    in.pose.position.z = position[2];

    // rotate
    arm_controller->rotate_from_camera_to_robot(in, out);

    // get the result
    point_robot_frame.push_back(out.pose.position.x);
    point_robot_frame.push_back(out.pose.position.y);
    point_robot_frame.push_back(out.pose.position.z);
}