#ifndef __MINIMUMDISTANCE3_H
#define __MINIMUMDISTANCE3_H

struct S_Point{ double x, y, z;};

class coordinate{
    public:
        struct S_Point ch1, ch2, ch3;   
        struct S_Point cy1, cy2, cy3, cy4, cy5, cy6;
        double d11, d12, d21, d22, d31, d32, d41, d42, d51, d52, d61, d62;
        double d1min, d2min, d3min, d4min, d5min, d6min;
        double dminsum;
        void cal();
        void print();        
        double get_dist();
};

#endif