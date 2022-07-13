/**
 * @file arm.h
 * @author Rover (you@domain.com)
 * @brief This class defines the elements for the arm
 * It is a struct in general
 * @version 0.1
 * @date 2022-07-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ARM_H
#define __ARM_H

#include "joint.h"


// this class stores the details of the human
class arm
{
public:
    arm(/* args */);
    ~arm();

    // data
    joint hip;
    joint shoulder;
    joint elbow;
    joint wrist;
    joint hand_thumb;
    joint hand_pinky;
    joint hand_index;
};

#endif
