#include "arm.h"
#include <cmath>
arm::arm(/* args */)
{
}

arm::~arm()
{
}

/**
 * @brief copy the arm
 * 
 * @param src 
 * @param dest 
 */
void arm::copy(arm* src, arm* dest) {
    joint::joint_copy(& src->hip, & dest->hip);
    joint::joint_copy(& src->shoulder, & dest->shoulder);
    joint::joint_copy(& src->elbow, & dest->elbow);
    joint::joint_copy(& src->wrist, & dest->wrist);
    joint::joint_copy(& src->hand_thumb, & dest->hand_thumb);
    joint::joint_copy(& src->hand_pinky, & dest->hand_pinky);
    joint::joint_copy(& src->hand_index, & dest->hand_index);
}

/**
 * @brief Get the arm length
 * Please make sure the data in elbow, wrist and shoulder is valid
 * @return double 
 */
double arm::get_arm_length() 
{
    double length = 0;

    length += std::sqrt(std::pow(wrist.get_x() - elbow.get_x(), 2)+
                        std::pow(wrist.get_y() - elbow.get_y(), 2)+
                        std::pow(wrist.get_z() - elbow.get_z(), 2));
    
    length += std::sqrt(std::pow(shoulder.get_x() - elbow.get_x(), 2)+
                        std::pow(shoulder.get_y() - elbow.get_y(), 2)+
                        std::pow(shoulder.get_z() - elbow.get_z(), 2));

    return length;
}


/**
 * @brief Get the arm positions 
 * 
 * @return std::vector<std::vector<double>> [wrist/elbow/shoulder]*[x,y,z]
 */
std::vector<std::vector<double>> arm::get_arm_positions()
{
    std::vector<std::vector<double>> res;
    
    std::vector<double> shoulder_position{shoulder.get_x(), shoulder.get_y(), shoulder.get_z()};
    res.push_back(shoulder_position);

    std::vector<double> elbow_position{elbow.get_x(), elbow.get_y(), elbow.get_z()};
    res.push_back(elbow_position);

    std::vector<double> wrist_position{wrist.get_x(), wrist.get_y(), wrist.get_z()};
    res.push_back(wrist_position);

    return res;
}