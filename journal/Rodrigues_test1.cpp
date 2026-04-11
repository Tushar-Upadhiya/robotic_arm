#include<iostream>
#include<cmath>
#include<Eigen/Dense>
#include "kinematics_utils.hpp"
int main(){
    Eigen::Vector3d joint_axis(0,0,1);
    double angle = M_PI/4.0;
    Eigen::Vector3d link_offset(1.0,0,0);

    Eigen::Matrix3d R = robo_math::rodrigues_rotation(joint_axis,angle);
    Eigen::Matrix4d T = robo_math::create_transform(R,link_offset);

    std::cout<<T<<std::endl;
    std::cout<<std::endl;
    return 0;
}