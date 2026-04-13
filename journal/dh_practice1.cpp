//The robot consists of a rotating base pedestal (Link 1) with a height of 0.5m and a 90-degree twist , followed by two planar links—an upper arm (Link 2) of 1.2m and a forearm (Link 3) of 1.0m.
#include<iostream>
#include<cmath>
#include<Eigen/Dense>
#include "kinematics_utils.hpp"

double deg_to_rad(double d){
double r = d*M_PI/180.0;
return r;
}

int main(){
    double theta1 = deg_to_rad(0);
    double theta2 = deg_to_rad(0);
    double theta3 = deg_to_rad(0);

    //transformation matrix for joint 1
    Eigen::Matrix4d T1 = robo_math::dh_transform(theta1,0.5,0.0, M_PI/2.0);

    //transformation matrix for joint 2
    Eigen::Matrix4d T2 = robo_math::dh_transform(theta2,0.0,1.2,0.0);

    //transformation matrix for joint 3
    Eigen::Matrix4d T3 = robo_math::dh_transform(theta3,0.0,1.0,0.0);

    //overall transformation from base to end-effector
    Eigen::Matrix4d T = T1*T2*T3;

    //Exact position of end effector
    Eigen::Vector3d pos = T.block<3,1>(0,3);

    std::cout<<"End Effector Position: "<<pos.transpose()<<std::endl;
    return 0;
}