/**
 * @file difference_calculator.h
 * @author Rover (you@domain.com)
 * @brief The interface to for pose comparison
 * @version 0.1
 * @date 2022-07-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DIFFERENCE_CALCULATOR_H
#define __DIFFERENCE_CALCULATOR_H

#include <vector>
class difference_calculator
{
private:
    /* data */
public:
    difference_calculator(/* args */);
    ~difference_calculator();

    /**
     * @brief Get the best index object
     * Robot joint positions and the human arm position would be 
     * given and the best robot pose index would be returned
     * @param robot_poses a groups of robot arm joint poses 
     * The format would be [number of solutions]*[number of joints]*[x,y,z]
     * @param arm_poses The human arm psoiton [number of joints]*[x,y,z]
     * @return int The index of the best solution
     */
    int get_best_index(std::vector<std::vector<std::vector<double>>> robot_poses, 
                                    std::vector<std::vector<double>> arm_poses);


};





#endif