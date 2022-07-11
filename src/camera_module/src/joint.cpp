#include "joint.h"

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
    x = _z;
}

bool joint::is_reliable() {
    return relibility;
}

void joint::set_reliable(bool state) {
    relibility = state;
}

