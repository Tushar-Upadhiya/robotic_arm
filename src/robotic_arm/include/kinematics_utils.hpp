#ifndef KINEMATICS_UTILS_HPP
#define KINEMATICS_UTILS_HPP

#include<cmath>
#include<Eigen/Dense>

 namespace robo_math{
    inline Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& k) {
        Eigen::Matrix3d K;
        K <<    0,      -k.z(),   k.y(),
              k.z(),       0,     -k.x(),
             -k.y(),     k.x(),      0;
        return K;
    }

    inline Eigen::Matrix3d rodrigues_rotation(Eigen::Vector3d axis, double theta) {
      
        axis.normalize();
        
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d K = skew_symmetric(axis);
        
       
        return I + std::sin(theta) * K + (1 - std::cos(theta)) * (K * K);
    }

    inline Eigen::Matrix4d create_transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& P) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = P;
        return T;
    }

    inline Eigen::Matrix4d transform_axis_angle(const Eigen::Vector3d& axis, double theta, const Eigen::Vector3d& translation) {
        Eigen::Matrix3d R = rodrigues_rotation(axis, theta);
        return create_transform(R, translation);
    }

    
    inline Eigen::Matrix4d translate(double x, double y, double z) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0, 3) = x;
        T(1, 3) = y;
        T(2, 3) = z;
        return T;
    }


    inline Eigen::Matrix4d dh_transform(double theta, double d, double a, double alpha) {
        Eigen::Matrix4d T;
        T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
             0,           sin(alpha),             cos(alpha),            d,
             0,           0,                      0,                     1;
        return T;
    }
 }

 #endif