#ifndef __DIFFERENCE_CALCULATOR_H
#define __DIFFERENCE_CALCULATOR_H

#include "arm_manager.h"
#include "camera_pose.h"
#include "MinimumDistance4.h"

class difference_calculator
{
private:
    /* data */
public:
    difference_calculator(/* args */);
    ~difference_calculator();

    int best_index = 0;
    int get_best_index(std::vector<std::vector<double>> robot_poses, std::vector<double> arm_poses);


};






#endif