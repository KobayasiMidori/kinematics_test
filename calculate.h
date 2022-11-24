//
// Created by kobayasi on 22-10-29.
//

#ifndef BAER_ETHERCAT_CALCULATE_H
#define BAER_ETHERCAT_CALCULATE_H

#define PI 3.14159265359

struct binary_qua {
    // 2 : x^2 + y^2 = h^2
    // 1 -> y = k*x + t
    double  h;
    double  k;
    double  t;
    double  con;
    bool    ero;
    double  output[2];

    void cal(int index);
};

double Sign(double x);
double Ramp(double dst, double v0, double inc);
double deg2rad(double x);


#endif //BAER_ETHERCAT_CALCULATE_H
