//
// Created by kobayasi on 22-10-29.
#include <iostream>
#include <fstream>
#include "calculate.h"
#include "cmath"



double Sign(double x){
    if (x < 0)
        return -1.0;
    else if (x > 0)
        return 1.0;
    else
        return 0.0;
}

double Ramp(double dst, double v0, double inc){
    if (fabs(v0 - dst) < inc)
        return v0;
    else
        return (v0 + Sign(dst - v0)*inc);
}

double deg2rad(double x)
{
    double a;
    a = x/180.0*PI;
    return a;
}

void binary_qua::cal(int index) {
    double a, b, c;
    double x[2], y[2];
    a = (1+k*k);
    b = 2*k*t;
    c = t*t-h*h;
    con = b*b - 4*a*c;
    if (con < 0)
        ero = true;
    else
        ero = false;
    x[0] = (-b + sqrt(con))/(2*a);
    x[1] = (-b - sqrt(con))/(2*a);

    y[0] = k*x[0] + t;
    y[1] = k*x[1] + t;

    output[0] = x[index];
    output[1] = y[index];
}
//
