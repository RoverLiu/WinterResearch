/**
 * @file joint.h
 * @author Rover (you@domain.com)
 * @brief save the joint details
 * @version 0.1
 * @date 2022-07-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __JOINT_H
#define __JOINT_H
#include <vector>
#include <iostream>

// this module handles each joint on the human body

class joint
{
private:
    double x;
    double y;
    double z;
    bool relibility;
public:
    joint(/* args */);
    ~joint();

    double get_x();
    double get_y();
    double get_z();

    void set_x(double _x);
    void set_y(double _y);
    void set_z(double _z);

    bool is_reliable();
    void set_reliable(bool state);

    /**
     * @brief copy the joint data
     * 
     * @param src 
     * @param dest 
     */
    static void joint_copy(std::vector<float> src, joint* dest);
    static void joint_copy(joint* src, joint* dest);
    static bool is_same(double a, double b);

};


#endif