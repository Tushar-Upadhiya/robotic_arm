#include<iostream>
#include<Eigen/Dense>
#include<cmath>
int main(){
    double theta=M_PI/2.0;
    Eigen::Matrix3d Rz;
    //Z-Rotation Matrix
    Rz << cos(theta), -sin(theta), 0,
          sin(theta),  cos(theta), 0,
          0,           0,           1;

    //X-Rotation Matrix
    Eigen::Matrix3d Rx;
    Rx << 1, 0,           0,
          0, cos(theta), -sin(theta),
          0, sin(theta),  cos(theta);

    //Fixed Axis rotation
    Eigen::Matrix3d R_fixed = Rx * Rz;

    //Moving Axix rotation
    Eigen::Matrix3d R_moving = Rz*Rx;

    //custom vector
    Eigen::Vector3d v(0,1,0);

    std::cout<<"Original vector: "<<v.transpose()<<std::endl;
    std::cout<<"Rotating around fixed axis: "<<(R_fixed*v).transpose()<<std::endl;
    std::cout<<"Rotating around moving axis: "<<(R_moving*v).transpose()<<std::endl;

    return 0;
}