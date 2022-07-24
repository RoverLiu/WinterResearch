#ifndef __MINIMUMDISTANCE4_H
#define __MINIMUMDISTANCE4_H

struct S_Point{ double x, y, z;};

class coordinate{
    public:
        struct S_Point ch1, ch2, ch3;   
        struct S_Point cy1, cy2, cy3, cy4, cy5, cy6;
        double h12, h23, y12, y23, y34, y45, y56;
        double y14, y46;
        double RatioUpper, RatioLower;
        double d11, d12, d21, d22, d31, d32, d41, d42, d51, d52, d61, d62;
        double d1min, d2min, d3min, d4min, d5min, d6min;
        double dminsum;
        void cal();
        void print();        
};

#endif