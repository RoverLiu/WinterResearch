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
    // camera = my_camera_manager->get_right_arm();
    // robot = my_arm_manager->get_right_arm();
    // shoulder = my_arm_manager->get_right_shoulder_position();
    // imitate_one_arm(camera, robot, shoulder);
    std::cout<<"--------finish imitation------------\n";

}

void control_manager::imitate_one_arm(arm* camera_arm, robot_arm_control* robot_arm, tf2::Vector3 shoulder) 
{
    if (camera_arm != NULL) {
        // get end pose in robot frame from camera 
        geometry_msgs::PoseStamped EEF_pose = 
            robot_arm->get_pose_in_robot_frame(camera_arm, shoulder);
        
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
        int index = my_calculator->get_best_index(positions, 
                                    camera_arm->get_arm_positions());

        // exectue the command    
        // robot_arm->move_to_arm_pos_by_id(index);
    }
    else {
        std::cout<<"ERROR: invalid camera pose received\n";
    }
} 


