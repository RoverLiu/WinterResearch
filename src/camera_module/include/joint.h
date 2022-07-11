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

};


