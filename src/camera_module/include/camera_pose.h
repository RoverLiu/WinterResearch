#include "arm.h"

// ----------------------------------------------------------
// this class is designed to process the pixel data and manage the 3D 
// pose of human for each joint
class camera_pose
{
private:
    /* data */
    arm left;
    arm right;
    
public:
    camera_pose(/* args */);
    ~camera_pose();
};



