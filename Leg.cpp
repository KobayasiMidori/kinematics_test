//
// Created by kobayasi on 22-11-9.
//
#include <iostream>
#include <fstream>

#include "Leg.h"

Length_Angle::Length_Angle() {
    OJ = 0.1;
    JK = 0.1;
    KB = 0.1;
    AC = 0.2;
    BE = 0.13;
    DG = 0.2;
    CD = 0.07;
    GH = 0.2;
    BD = AC;
    AB = CD;
    EF = DG;
    ED = AC - BE;

    JO = OJ;
    KJ = JK;
    BK = KB;
    CA = AC;
    EB = BE;
    GD = DG;
    DC = CD;
    HG = GH;
    DB = BD;
    BA = AB;
    FE = EF;
    DE = ED;

}


KinematicsData::KinematicsData(right_left tt){
    std::string name[7] = {"roll", "yaw", "pitch", "knee", "pas1", "pas2", "ankle"};
    std::string point[7] = {"OJ", "JK", "KBD", "AB", "CDG", "FGH", "Hh"};

    if (tt == right)    sign = 1;
    else        sign = -1;

    for (int i; i < 7; i++)
    {
        link[i].ID = i;
        link[i].point = point[i];
        link[i].name = name[i];
        link[i].p << 0, 0, 0;
        link[i].R << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        link[i].R0 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
        link[i].v << 0, 0, 0;
        link[i].w << 0, 0, 0;
        link[i].q = 0;
        link[i].dq = 0;
        link[i].ddq = 0;
        link[i].a << 0, 0, 0;
        link[i].b << 0, 0, 0;
        link[i].vertex << 0, 0, 0;
        link[i].face = 0;
        link[i].m = 0;
        link[i].c << 0, 0, 0;
        link[i].I << 0, 0, 0;
    }
    link[0].p << 0, 0, 0;

    link[0].R0 << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    link[1].R0 << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    if (tt == right)    link[2].R0 << 0, 1, 0, 0, 0, -1, -1, 0, 0;
    else    link[2].R0 << 0, -1, 0, 0, 0, 1, -1, 0, 0;
    link[3].R0 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    link[4].R0 << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    link[5].R0 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    link[6].R0 << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    link[0].a << -1, 0, 0;
    link[1].a << -1, 0, 0;
    if (tt == right)    link[2].a << 0, -1, 0;
    else    link[2].a << 0, 1, 0;
    link[3].a << 0, 0, 1;
    link[4].a << 0, 0, 1;
    link[5].a << 0, 0, 1;
    link[6].a << 0, 0, 1;

    link[0].b << 0, 0, 0;
    link[1].b << 0, 0, -A.OJ;
    link[2].b << 0, 0, -A.JK;
    link[3].b << 0, 0, -A.KB;
    link[4].b << A.BD, 0, -A.KB;
    link[5].b << A.DG, 0, 0;
    link[6].b << A.HG, 0, 0;
}

void KinematicsData::FKinematics(double* theta){

    std::cout << "=======================FK==============================" << std::endl;

    link[0].q = theta[0];//deg2rad(12);
    link[1].q = theta[1];//deg2rad(20);
    link[2].q = theta[2];//deg2rad(-10);
    link[3].q = theta[3];//deg2rad(120);
    link[4].q = theta[4];//deg2rad(-3);
    link[5].q = theta[5];//deg2rad(5);
    link[6].q = theta[6];//deg2rad(90);

    link[0].p << 0, 0, 0;
    std::cout << "p0：" << std::endl << link[0].p << std::endl;
    link[0].R = link[0].R0*Rz3(link[0].q);

    link[1].p = link[0].p + link[0].R*link[1].b;//(A.OJ, 0, 0);
    std::cout << "p1" << std::endl << link[1].p << std::endl;
    link[1].R = link[0].R*link[1].R0*Rz3(link[1].q);//Vec3<double> Vec_roll = Rx3(-1*roll)*Vec3<double>(0, 0, -1);

    link[2].p = link[1].p + link[1].R*link[2].b;//Vec3<double> PK = PJ + A.JK*Vec_roll;
    std::cout << "p2" << std::endl << link[2].p << std::endl;
    link[2].R = link[1].R*link[2].R0*Rz3(link[2].q);//Vec3<double> Vec_yaw = Rx3(-1*roll)*Rz3(-1*yaw)*Vec3<double>(0, 1, 0);

    link[3].p = link[2].p + link[2].R*link[3].b;//Vec3<double> PB = PK + A.KB*Vec_yaw;
    std::cout << "p3" << std::endl << link[3].p << std::endl;
    link[3].R = link[2].R*link[3].R0*Rz3(sign*link[3].q);//Vec3<double> va1 = Ry3(-1*theta1)*Vec3<double>(0, 0, -1);Vec3<double> vb1 = Ry3(-1*theta2)*va1;

    link[4].p = link[2].p + link[2].R*link[4].b;
    std::cout << "p4" << std::endl << link[4].p << std::endl;
    link[4].R = link[3].R*link[4].R0*Rz3(sign*link[4].q);

    link[5].p = link[4].p + link[4].R*link[5].b;
    std::cout << "p5" << std::endl << link[5].p << std::endl;
    link[5].R = link[2].R*link[5].R0*Rz3(sign*link[5].q);

    link[6].p = link[5].p + link[5].R*link[6].b;
    std::cout << "p6" << std::endl << link[6].p << std::endl;
    link[6].R = link[5].R*link[6].R0*Rz3(sign*link[6].q);

    x = link[6].p(0);
    y = link[6].p(1);
    z = link[6].p(2);
    p = link[6].q;
    yaw_ik = atan(sin(link[1].q)/cos(link[0].q)/cos(link[1].q));
    std::cout << "yaw_ik" << std::endl << yaw_ik << std::endl;
    roll_ik = asin(sin(link[0].q)*cos(link[1].q));

}

void KinematicsData::IKinematics(double* PH ,double yaw_ik_in, double p_in, double pas1, double pas2){
    std::cout << "=======================IK==============================" << std::endl;

    x = PH[0];//-0.162944;
    y = PH[1];//-0.104843;
    z = PH[2];//-0.544781;
    yaw_ik = yaw_ik_in;//0.356227;// = atan(sin(yaw_fk)/cos(roll_fk)/cos(yaw_fk));
    p = p_in;//deg2rad(10.0);// = p_fk;
    link[4].q = pas1;//deg2rad(-3);
    link[5].q = pas2;//deg2rad(5);

    link[6].p << x, y, z;

    link[0].p << 0, 0, 0;
    link[0].R = link[0].R0*Rz3(link[0].q);
    link[1].p = link[0].p + link[0].R*link[1].b;//(A.OJ, 0, 0);
    //yaw_ik: #1世界坐标系 P~1, #0->#1
    Vec3<double> Pj1 = Rz3(-yaw_ik)*link[1].p;
    Vec3<double> Ph1 = Rz3(-yaw_ik)*link[6].p;

    // va = [y; z];
    // vb = [-z; y];
    // Pj1 = [ yj; zj];
    // Ph1 = [ yh; zh];
    // Pk1 = Pj1 + JK*va;
    // Pb1 = Pk1 + KB*vb;
    // bh1 = Ph1 - Pb1;
    // bh1y = bh1(1);%yh - yj - JK*y + KB*z
    // bh1z = bh1(2);%zh - zj - JK*z - KB*y
    // bh1//jk1
    // 1: bh1y/bh1z = (yh - yj - JK*y + KB*z)/(zh - zj - JK*z - KB*y) = y/z
    // 2: y^2 + z^2 = 1
    // right: 1->y = (yh - yj)/(zh - zj)*z + KB/(zh - zj)  =kz + t
    // left : 1->y = (yh - yj)/(zh - zj)*z - KB/(zh - zj)  =kz + t
    // yj = Pj1(2);
    // zj = Pj1(3);
    // yh = Ph1(2);
    // zh = Ph1(3);
    binary_qua_cal.h = 1;
    binary_qua_cal.k = (Ph1(1) - Pj1(1))/(Ph1(2) - Pj1(2));
    binary_qua_cal.t = sign*abs(link[3].b.norm())/(Ph1(2) - Pj1(2));
    binary_qua_cal.cal(1);

    double z1 = binary_qua_cal.output[0];
    double y1 = binary_qua_cal.output[1];
    //V2->V3 #1->#0
    Vec3<double> vb0 = Rz3(yaw_ik)*Vec3<double>(0, -z1, y1);
    Vec3<double> vb10  = Rz3(yaw_ik)*Vec3<double>(0, 1, 0);
    double tmp = vb0.transpose()*vb10;
    double roll_ik = acos(tmp/vb0.norm()/vb10.norm());
    if (y1 > 0)   roll_ik = -roll_ik;
    link[0].q = atan(sin(roll_ik)/cos(roll_ik)/cos(yaw_ik));
    link[1].q = asin(sin(yaw_ik)*cos(roll_ik));

    link[0].R = link[0].R0*Rz3(link[0].q);
    link[1].R = link[0].R*link[1].R0*Rz3(link[1].q);
    link[2].p = link[1].p + link[1].R*link[2].b;
    link[2].R = link[1].R*link[2].R0*Rz3(link[2].q);

    link[3].p = link[2].p + link[2].R*link[3].b;
    std::cout<<"link[3].p = "<< link[3].p <<std::endl;
    //坐标系#2,->roll ->yaw,腿平面,x#3 = -x#0, y#3 = z#0
    Vec3<double> Pb2_3 (0, 0, 0);
    std::cout<<"link[6].p - link[3].p = "<< link[6].p - link[3].p <<std::endl;
    Vec3<double> Ph2_3 = Rz3(-link[1].q)*Rx3(link[0].q)*(link[6].p - link[3].p);
    std::cout<<"Ph2_3 = "<< Ph2_3 <<std::endl;
    //v3->v2,
    Vec2<double> Pb2, Ph2;
    Pb2 << -Pb2_3(0), Pb2_3(2);
    Ph2 << -Ph2_3(0), Ph2_3(2);
    std::cout<<"Ph2 = "<< Ph2 <<std::endl;

    double h2 = abs(Ph2.norm());

    //坐标系#3, pitch为0
    // 1 : (x + GH*sin(theta_G))^2 + (y + GH*cos(theta_G) + BD)^2 = DG^2
    // 2 : x^2 + y^2 = h2^2
    // 1 -> y = k*x + t
    binary_qua_cal.h = h2;
    binary_qua_cal.k = -2*A.GH*sin(link[5].q)/(2*A.GH*cos(link[5].q) + 2*A.BD);
    binary_qua_cal.t = (A.DG*A.DG - h2*h2 - A.GH*A.GH - A.BD*A.BD - 2*A.BD*A.GH*cos(link[5].q))/(2*A.GH*cos(link[5].q) + 2*A.BD);
    binary_qua_cal.cal(0);

    double x3 = binary_qua_cal.output[0];
    double y3 = binary_qua_cal.output[1];
    Vec2<double> Ph3 (x3, y3);
    Vec2<double> Pg3 = Ph3 + Rz(-link[5].q)*Vec2<double>(0,A.GH);
    Vec2<double> Pb3 (0, 0);
    Vec2<double> Pd3 (0, -A.BD);
    Vec2<double> vgd3 = Pd3 - Pg3;
    Vec2<double> vba3 = Rz(link[4].q)*vgd3;
    Vec2<double> vbd3 = Pd3 - Pb3;
    tmp = vba3.transpose()*vbd3;
    link[3].q = acos(tmp/vba3.norm()/vbd3.norm());
    Vec2<double> vbh2 = Ph2 - Pb2;
    std::cout << "vbh2 = " << vbh2 << std::endl;
    Vec2<double> vbh3 = Ph3 - Pb3;
    std::cout << "vbh3 = " << vbh3 << std::endl;
    link[2].q = sign*(asin(vbh3(0)/vbh3.norm()) - asin(vbh2(0)/vbh2.norm()));
    link[6].q = p;

    std::cout << "roll  =  " << link[0].q*180/PI << std::endl;
    std::cout << "yaw  =  " << link[1].q*180/PI << std::endl;
    std::cout << "pitch  =  " << link[2].q*180/PI << std::endl;
    std::cout << "knee  =  " << link[3].q*180/PI << std::endl;
    std::cout << "ankle  =  " << link[6].q*180/PI << std::endl;
}

Mat3<double> KinematicsData::Rx3(double theta){
    // for 2D-XY vector, rotation matrix along z axis
    Mat3<double> M;
    M << 1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta),  cos(theta);
    return M;
}

Mat3<double> KinematicsData::Ry3(double theta){
    // for 2D-XY vector, rotation matrix along z axis
    Mat3<double> M;
    M << cos(theta), 0, sin(theta),
            0,             1,               0,
            -sin(theta), 0, cos(theta);
    return M;
}

Mat3<double> KinematicsData::Rz3(double theta){
    // for 2D-XY vector, rotation matrix along z axis
    Mat3<double> M;
    M << cos(theta), -sin(theta), 0,
            sin(theta), cos(theta),  0,
            0,             0,              1;
    return M;
}

Mat2<double> KinematicsData::Rz(double theta){
    // for 2D-XY vector, rotation matrix along z axis
    Mat2<double> M;
    M << cos(theta),-sin(theta),
    sin(theta), cos(theta);
    return M;

}

Mat3<double> KinematicsData::cross_product(Vec3<double> a){
    Mat3<double> aa;
    aa << 0, -a(2), a(1),
          a(2), 0, -a(0),
          -a(1), a(0), 0;

    return aa;
}

Mat3<double> KinematicsData::Rodrigues(Vec3<double> a, double theta){
    Mat3<double> R;
    Mat3<double> E;

    E << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    R = E + cross_product(a)*sin(theta) + cross_product(a)*cross_product(a)*(1-cos(theta));

    return R;
}

