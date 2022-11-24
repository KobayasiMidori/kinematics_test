#include <iostream>
#include <fstream>
#include <string>
#include "Leg.h"

int main()
{
    KinematicsData FK(right);
    KinematicsData IK(right);
    double theta[7] = {deg2rad(-10), deg2rad(-20), deg2rad(-1), deg2rad(124), deg2rad(3), deg2rad(-5), deg2rad(90)};//;
    FK.FKinematics(theta);
    double PH[3] = {FK.x, FK.y, FK.z};
    double yaw_ik_in = FK.yaw_ik;// = atan(sin(yaw_fk)/cos(roll_fk)/cos(yaw_fk));
    double p_in = FK.p;// = p_fk;
    double pas1 = FK.link[4].q;
    double pas2 = FK.link[5].q;
    IK.IKinematics(PH ,yaw_ik_in, p_in, pas1, pas2);
/*
    KinematicsData FK(right);
    KinematicsData IK(right);
    double theta[7] = {deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(90), deg2rad(0), deg2rad(0), deg2rad(90)};//;
    FK.FKinematics(theta, right);
    double PH[3] = {FK.x, FK.y, FK.z};
    double yaw_ik_in = FK.yaw_ik;// = atan(sin(yaw_fk)/cos(roll_fk)/cos(yaw_fk));
    double p_in = FK.p;// = p_fk;
    double pas1 = FK.link[4].q;
    double pas2 = FK.link[5].q;
    IK.IKinematics(PH ,yaw_ik_in, p_in, pas1, pas2, right);
    */
}

