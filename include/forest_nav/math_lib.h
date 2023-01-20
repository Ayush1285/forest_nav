#ifndef MATH_LIB_H
#define MATH_LIB_H

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <utility>
#include "custom_types.h"

inline double calNorm(const GridPoint& node0, const GridPoint& node1)
{
    return std::sqrt(std::pow(node1.x - node0.x, 2) + std::pow(node1.y - node0.y, 2));
}

double calculateIntegral(double lb, double ub, double(*fx)(double))
{
    float stepsize = 0.01;
    int steps = std::floor((ub - lb)/stepsize);
    double sum = 0;
    for(int i=1; i <= steps; i++)
    {
        sum += fx(lb + stepsize*(2*i - 1)/2) * stepsize;
    }
    return sum;
}



namespace ConvexCorridor
{
    inline double cal_scost(int x){
        return (-103/(1 + std::exp(-0.55*x+3.5))) + 103;
        //return (-50/(1 + std::exp(-0.25*x+3.5))) + 50;
    }
    inline double cal_ocost(double x){
        return (-50/(1 + std::exp(-0.7*x+8))) + 50;
    }
    inline double cal_icost(double x){
        return (-10/(1 + std::exp(-2*x+8))) + 10;
    }
}


#endif