// Copyright 2021 demax
#ifndef INCLUDE_COMMON_GLOBAL_H_
#define INCLUDE_COMMON_GLOBAL_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

const double kPi = 3.141592653589793;
const Eigen::AngleAxisd V3d2AngleAxisd(const Eigen::Vector3d v3d);
const Eigen::Quaterniond RotationVector2Quaternion(const Eigen::Vector3d& v3d);
const Eigen::Matrix3d V3d2Skew(const Eigen::Vector3d& vec);
Eigen::Quaterniond Euler2Quaternion(const Eigen::Vector3d& euler);
Eigen::Vector3d Quaternion2Euler(const Eigen::Quaterniond& qua);

// struct GLOBAL
// {
//     const double Re = 6378137;
//     const double f = 1.0/298.257;
//     const double wie = 7.2921151467e-5;
//     const double Rp = (1.0-f)*Re;
//     const double e = sqrtf(2*f-f*f);
//     const double e2 = e*e;
//     const double ep = sqrtf(Re*Re)/Rp;
//     const double ep2 = ep*ep;
//     const double g0 = 9.7803267714;
//     const double mg = 1.0e-3*g0;
//     const double ug = 1.0e-6*g0;
//     const double deg = pi/180;
//     const double min = deg/60;
//     const double sec = min/60;
//     const double hour = 3600.0;
//     const double dps = deg;
//     const double dph = dps/hour;
//     const double dpsh = deg/sqrtf(hour);
//     const double dphpsh = dph/sqrtf(hour);
// };

#endif  // INCLUDE_COMMON_GLOBAL_H_
