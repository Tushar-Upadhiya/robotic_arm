#include<iostream>
#include<cmath>
#include<Eigen/Dense>
int main(){
    Eigen::Vector3d k(1.0,1.0,0.0);
    std::cout<<"Matrix k before normalization."<<std::endl;
    std::cout<<k.transpose()<<std::endl;
    std::cout<<std::endl;

    k.normalize();
    std::cout<<"After normalization."<<std::endl;
    std::cout<<k.transpose()<<std::endl;
    std::cout<<std::endl;

    double theta = M_PI/2.0;

    Eigen::Matrix3d K;
    K <<    0,      -k.z(),   k.y(),
          k.z(),       0,     -k.x(),
         -k.y(),     k.x(),      0;

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = I+K*(std::sin(theta))+(K*K)*(1-std::cos(theta));

    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = R * v;

    std::cout << "Rotation Axis (k): " << k.transpose() << std::endl;
    std::cout << "Rotation Matrix (R):\n" << R << std::endl;
    std::cout << "\nOriginal Vector: " << v.transpose() << std::endl;
    std::cout << "Rotated Vector:  " << v_rotated.transpose() << std::endl;

    return 0;
}