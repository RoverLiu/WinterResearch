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


