#include "difference_calculator.h"
#include <vector>

difference_calculator::difference_calculator(/* args */){
      
}

difference_calculator::~difference_calculator(){

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
    int row = robot_poses.size();
    int column = robot_poses[0].size();
    column = 6; //////////////

    int best_index = 0;
    struct S_Point ch1, ch2, ch3; 
    ch1.x = arm_poses[0][0]; ch1.y = arm_poses[0][1]; ch1.z = arm_poses[0][2];
    ch2.x = arm_poses[1][0]; ch2.y = arm_poses[1][1]; ch2.z = arm_poses[1][2];
    ch3.x = arm_poses[2][0]; ch3.y = arm_poses[2][1]; ch3.z = arm_poses[2][2];

    for (int i=0; i<row; i++){
        double dmin = 1000;
        struct S_Point cy1, cy2, cy3, cy4, cy5, cy6;
        cy1.x = robot_poses[i][0][0]; cy1.y = robot_poses[i][0][1]; cy1.z = robot_poses[i][0][2];
        cy2.x = robot_poses[i][1][0]; cy2.y = robot_poses[i][1][1]; cy2.z = robot_poses[i][1][2];
        cy3.x = robot_poses[i][2][0]; cy3.y = robot_poses[i][2][1]; cy3.z = robot_poses[i][2][2];
        cy4.x = robot_poses[i][3][0]; cy4.y = robot_poses[i][3][1]; cy4.z = robot_poses[i][3][2];
        cy5.x = robot_poses[i][4][0]; cy5.y = robot_poses[i][4][1]; cy5.z = robot_poses[i][4][2];
        cy6.x = robot_poses[i][5][0]; cy6.y = robot_poses[i][5][1]; cy6.z = robot_poses[i][5][2];
        class coordinate test1;
        test1.ch1 = ch1; test1.ch2 = ch2; test1.ch3 = ch3; 
        test1.cy1 = cy1; test1.cy2 = cy2; test1.cy3 = cy3; 
        test1.cy4 = cy4; test1.cy5 = cy5; test1.cy6 = cy6;
        if (dmin > test1.dminsum){
            dmin = test1.dminsum;
            best_index = i;
        }
    }
    return best_index;
}

