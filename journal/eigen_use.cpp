//using eigen library 
#include <iostream>
#include<Eigen/Dense>
#include<cmath>
int main(){
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::cout<<"Creating a 4x4 identity Matrix: "<<std::endl;
    std::cout<<T<<std::endl;
    std::cout<<std::endl;

    T(0,3)=1.0;
    T(2,3)=0.5;

    double theta = M_PI/2.0;
    Eigen::Matrix3d rotation;
    rotation= Eigen::AngleAxisd(theta,Eigen::Vector3d::UnitY());

    T.block<3,3>(0,0)=rotation;
    std::cout<<T<<std::endl;
    std::cout<<std::endl;

    Eigen::Vector4d local_point(0.2,0.0,0.0,1.0);
    Eigen::Vector4d global_point = T*local_point;
    std::cout<<local_point.transpose()<<std::endl;
    std::cout<<std::endl;
    std::cout<<global_point.transpose()<<std::endl;


    return 0;
}