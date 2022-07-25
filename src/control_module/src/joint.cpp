#include "joint.h"
#include <vector>
#include <cmath>

joint::joint(/* args */)
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    relibility = false;
}

joint::~joint()
{
}

double joint::get_x() 
{
    return x;
}

double joint::get_y() 
{
    return y;
}

double joint::get_z() 
{
    return z;
}

void joint::set_x(double _x) {
    x = _x;
}

void joint::set_y(double _y) {
    y = _y;
}

void joint::set_z(double _z) {
    z = _z;
}

bool joint::is_reliable() {
    return relibility;
}

void joint::set_reliable(bool state) {
    relibility = state;
}

/**
 * @brief copy the joint data
 * 
 * @param src 
 * @param dest 
 */
void joint::joint_copy(std::vector<float> src, joint* dest)
{
    // invalid
    if (is_same(src[0], 0.0) 
        && is_same(src[1], 0.0)
        && is_same(src[2], -1.0)) 
    {
        dest->set_reliable(false);
    }
    else {
        // valid
        dest->set_x(src[0]);
        dest->set_y(-src[1]);
        dest->set_z(-src[2]);
        dest->set_reliable(true);
    }
}

void joint::joint_copy(joint* src, joint* dest) 
{
    dest->set_x(src->get_x());
    dest->set_y(src->get_y());
    dest->set_z(src->get_z());
    dest->set_reliable(src->is_reliable());
}


// compare two double number
bool joint::is_same(double a, double b)
{
    const double EPSILON = 0.0001;
    return fabs(a - b) < EPSILON;
}
