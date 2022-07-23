#include "difference_calculator.h"
#include <vector>

difference_calculator::difference_calculator(/* args */)
{
}

difference_calculator::~difference_calculator()
{
}

 /**
 * @brief Get the best index object
 * Robot joint positions and the human arm position would be 
 * given and the best robot pose index would be returned
 * @param robot_poses a groups of robot arm joint poses 
 * The format would be [number of solutions]*[number of joints]*[x,y,z]
 * @param arm_poses The human arm psoiton [number of joints]*[x,y,z]
 * @return int The index of the best solution
 */
int difference_calculator::get_best_index(std::vector<std::vector<std::vector<double>>> robot_poses, 
                                                        std::vector<std::vector<double>> arm_poses) 
{
    
}