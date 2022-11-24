//
// Created by kobayasi on 22-11-9.
//

#ifndef BAER_ETHERCAT_LEG_H
#define BAER_ETHERCAT_LEG_H

#include "calculate.h"
#include "cppTypes.h"

enum right_left {
    right = 0,
    left = 1
};

struct Length_Angle{
    double  OJ;
    double  JK;
    double  KB;
    double  AC;
    double  BE;
    double  DG;
    double  CD;
    double  GH;
    double  CDG;
    double  FGH;
    double  BD;
    double  AB;
    double  EF;
    double  ED;

    double  JO;
    double  KJ;
    double  BK;
    double  CA;
    double  EB;
    double  GD;
    double  DC;
    double  HG;
    double  GDC;
    double  HGF;
    double  DB;
    double  BA;
    double  FE;
    double  DE;

    double  theta_D;
    double  theta_G;
    Length_Angle();
};

struct unit_link {
    int     ID;
    std::string     name;
    std::string     point;
    Vec3<double>    p;
    Mat3<double>    R;
    Mat3<double>    R0;
    Vec3<double>    v;
    Vec3<double>    w;
    double          q;
    double          dq;
    double          ddq;
    Vec3<double>    a;
    Vec3<double>    b;
    Vec3<double>    vertex;
    double          face;
    double          m;
    Vec3<double>    c;
    Vec3<double>    I;
};

class KinematicsData {
public:
    KinematicsData(right_left tt);
    void FKinematics(double* theta);
    void IKinematics(double* PH ,double yaw_ik, double p, double pas1, double pas2);

    Mat3<double> Rx3(double theta);
    Mat3<double> Ry3(double theta);
    Mat3<double> Rz3(double theta);
    Mat2<double> Rz(double theta);

    Mat3<double> Rodrigues(Vec3<double> a, double theta);

    Mat3<double> cross_product(Vec3<double> a);
//protected:
    Length_Angle    A;
    unit_link   link[7];

    double  sign;
    binary_qua binary_qua_cal;
    double  x;
    double  y;
    double  z;
    double  p;

    double  yaw_ik;
    double  roll_ik;

    bool IK_error1;
    bool IK_error2;
};

#endif //BAER_ETHERCAT_LEG_H
