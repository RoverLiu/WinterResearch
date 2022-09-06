//ch means the Coordinate of Human arm
//cy means the Coordinate of Yumi arm
#include "MinimumDistance4.h"
#include <iostream>
#include <math.h>
using namespace std;

double DistanceOfPointToLine(S_Point a, S_Point b, S_Point s){
    double ab = sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
    double as = sqrt(pow((a.x - s.x), 2) + pow((a.y - s.y), 2) + pow((a.z - s.z), 2));
    double bs = sqrt(pow((s.x - b.x), 2) + pow((s.y - b.y), 2) + pow((s.z - b.z), 2));
    //cout << "##" << as << "##"  << bs << "##" << ab <<endl;

    if (as == 0 || bs == 0){
        return 0;
    }
    else{
        double cos_A = (pow(as,2)+pow(ab,2)-pow(bs,2))/(2*ab*as);    
        double sin_A = sqrt (1 - pow(cos_A,2));
        return as*sin_A;
    }
}

double DistanceOfDots(S_Point d1, S_Point d2){
    return sqrt(pow((d1.x - d2.x), 2) + pow((d1.y - d2.y), 2) + pow((d1.z - d2.z), 2));
}

void coordinate::cal() { //calculation

    //////////////////////    Normalisation  ///////////////////////////
    ch1.x = ch1.x - ch1.x; ch1.y = ch1.y - ch1.y; ch1.z = ch1.z - ch1.z;
    ch2.x = ch2.x - ch1.x; ch2.y = ch2.y - ch1.y; ch2.z = ch2.z - ch1.z;
    ch3.x = ch3.x - ch1.x; ch3.y = ch3.y - ch1.y; ch3.z = ch3.z - ch1.z;
    cy1.x = cy1.x - cy1.x; cy1.y = cy1.y - cy1.y; cy1.z = cy1.z - cy1.z;
    cy2.x = cy2.x - cy1.x; cy2.y = cy2.y - cy1.y; cy2.z = cy2.z - cy1.z;
    cy3.x = cy3.x - cy1.x; cy3.y = cy3.y - cy1.y; cy3.z = cy3.z - cy1.z;
    cy4.x = cy4.x - cy1.x; cy4.y = cy4.y - cy1.y; cy4.z = cy4.z - cy1.z;
    cy5.x = cy5.x - cy1.x; cy5.y = cy5.y - cy1.y; cy5.z = cy5.z - cy1.z;
    cy6.x = cy6.x - cy1.x; cy6.y = cy6.y - cy1.y; cy6.z = cy6.z - cy1.z;

    //////////////////// Getting Distance between joints ///////////////
    h12 = DistanceOfDots(ch1, ch2);  h23 = DistanceOfDots(ch2, ch3);
    y12 = DistanceOfDots(cy1, cy2);  y23 = DistanceOfDots(cy2, cy3);
    y34 = DistanceOfDots(cy3, cy4);  y45 = DistanceOfDots(cy4, cy5);
    y56 = DistanceOfDots(cy5, cy6); 
    y14 = DistanceOfDots(cy1, cy4);  y46 = DistanceOfDots(cy4, cy6);
    RatioUpper = y14 / h12; RatioLower = y46 / h23; 
    ch2.x = ch2.x + (ch2.x - ch1.x) * RatioUpper;
    ch2.y = ch2.y + (ch2.y - ch1.y) * RatioUpper;
    ch2.z = ch2.z + (ch2.z - ch1.z) * RatioUpper;
    ch3.x = ch3.x + (ch3.x - ch2.x) * RatioLower;
    ch3.y = ch3.y + (ch3.y - ch2.y) * RatioLower;
    ch3.z = ch3.z + (ch3.z - ch2.z) * RatioLower;

    d11 = DistanceOfPointToLine(ch1, ch2, cy1);
    d12 = DistanceOfPointToLine(ch2, ch3, cy1); 
    d1min = min(d11,d12); d1min = d11;
    d21 = DistanceOfPointToLine(ch1, ch2, cy2);
    d22 = DistanceOfPointToLine(ch2, ch3, cy2); 
    d2min = min(d21,d22); d2min = d21;
    d31 = DistanceOfPointToLine(ch1, ch2, cy3);
    d32 = DistanceOfPointToLine(ch2, ch3, cy3); 
    d3min = min(d31,d32); d3min = d31;
    d41 = DistanceOfPointToLine(ch1, ch2, cy4);
    d42 = DistanceOfPointToLine(ch2, ch3, cy4); 
    d4min = min(d41,d42); d4min = d42;
    d51 = DistanceOfPointToLine(ch1, ch2, cy5);
    d52 = DistanceOfPointToLine(ch2, ch3, cy5); 
    d5min = min(d51,d52); d5min = d52;
    d61 = DistanceOfPointToLine(ch1, ch2, cy6);
    d62 = DistanceOfPointToLine(ch2, ch3, cy6); 
    d6min = min(d61,d62); d6min = d62;

    dminsum = d1min + d2min +d3min +d4min +d5min +d6min;
}

void coordinate::print(){ 

    std::cout << "d11 = " << d11 << "; d12 = " << d12 << "; d1min = " << d1min <<std::endl; 
    std::cout << "d21 = " << d21 << "; d22 = " << d22 << "; d2min = " << d2min <<std::endl; 
    std::cout << "d31 = " << d31 << "; d32 = " << d32 << "; d3min = " << d3min <<std::endl; 
    std::cout << "d41 = " << d41 << "; d42 = " << d42 << "; d4min = " << d4min <<std::endl; 
    std::cout << "d51 = " << d51 << "; d52 = " << d52 << "; d5min = " << d5min <<std::endl; 
    std::cout << "d61 = " << d61 << "; d62 = " << d62 << "; d6min = " << d6min <<std::endl; 

    cout << ch2.x << endl;
    std::cout << "Minimum sum distance: " << dminsum << std::endl;

}