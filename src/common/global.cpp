#include "common/global.h"
const Eigen::AngleAxisd V3d2AngleAxisd(const Eigen::Vector3d v3d){
    double angle = v3d.norm();
    Eigen::Vector3d axis = v3d/angle;
    return Eigen::AngleAxisd(angle,axis);
}

const Eigen::Quaterniond RotationVector2Quaternion(const Eigen::Vector3d v3d){
    double n = v3d.norm();
    double n2 = n*n;
    double q0,q1,q2,q3;
    double s;
    if(n2 < 10.e-8){
        q0 = 1.0 - n2*(1.0/8.0 - (n2/384.0));
        s = 0.5 - n2*(1.0/48.0 - n2/3840.0);
    }
    else{
        double n_2 = n/2.0;
        q0 = cos(n_2);
        s = sin(n_2)/n;
    }
    return Eigen::Quaterniond(q0,s*v3d(0),s*v3d(1),s*v3d(2));
}