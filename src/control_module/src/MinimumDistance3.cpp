//ch means the Coordinate of Human arm
//cy means the Coordinate of Yumi arm
#include "MinimumDistance3.h"
#include <iostream>
#include <math.h>
using namespace std;



double DistanceOfPointToLine(S_Point a, S_Point b, S_Point s){
    double ab = sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
    double as = sqrt(pow((a.x - s.x), 2) + pow((a.y - s.y), 2) + pow((a.z - s.z), 2));
    double bs = sqrt(pow((s.x - b.x), 2) + pow((s.y - b.y), 2) + pow((s.z - b.z), 2));
    double cos_A = (pow(as,2)+pow(ab,2)-pow(as,2))/(2*ab*as);
    double sin_A = sqrt (1 - pow(cos_A,2));
    return as*sin_A;
}



void coordinate::cal() { //calculation
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
    /*
    std::cout << "d11 = " << d11 << "; d12 = " << d12 << "; d1min = " << d1min <<std::endl; 
    std::cout << "d21 = " << d21 << "; d22 = " << d22 << "; d2min = " << d2min <<std::endl; 
    std::cout << "d31 = " << d31 << "; d32 = " << d32 << "; d3min = " << d3min <<std::endl; 
    std::cout << "d41 = " << d41 << "; d42 = " << d42 << "; d4min = " << d4min <<std::endl; 
    std::cout << "d51 = " << d51 << "; d52 = " << d52 << "; d5min = " << d5min <<std::endl; 
    std::cout << "d61 = " << d61 << "; d62 = " << d62 << "; d6min = " << d6min <<std::endl; 
    */
    std::cout << "Minimum sum distance: " << dminsum << std::endl;

}

int main(){
    ///Given 9 structures
    struct S_Point ch1, ch2, ch3; //ch means the Coordinate of Human arm
    ch1.x = 2.3; ch1.y = 3.4; ch1.z = 4.2;
    ch2.x = 3.3; ch2.y = 4.2; ch2.z = 6.2;
    ch3.x = 4.3; ch3.y = 6.4; ch3.z = 8.2;
    struct S_Point cy1, cy2, cy3, cy4, cy5, cy6;//cy means the Coordinate of Yumi arm
    
    if (true){
        cy1.x = 1.2; cy1.y = 1.2; cy1.z = 1.6;
        cy2.x = 1.3; cy2.y = 2.2; cy2.z = 1.5;
        cy3.x = 1.4; cy3.y = 1.2; cy3.z = 1.4;
        cy4.x = 1.5; cy4.y = 2.2; cy4.z = 1.3;
        cy5.x = 1.6; cy5.y = 1.2; cy5.z = 1.2;
        cy6.x = 1.7; cy6.y = 2.2; cy6.z = 1.1;
    }

    class coordinate test1;
    if (true){
        test1.ch1 = ch1; test1.ch2 = ch2; test1.ch3 = ch3; 
        test1.cy1 = cy1; test1.cy2 = cy2; test1.cy3 = cy3; 
        test1.cy4 = cy4; test1.cy5 = cy5; test1.cy6 = cy6;
    }
         
    test1.cal();
    test1.print();
    
    return 0;
}




    