// Copyright 2021 demax
#include "common/global.h"
const Eigen::AngleAxisd V3d2AngleAxisd(const Eigen::Vector3d v3d) {
  double angle = v3d.norm();
  Eigen::Vector3d axis = v3d / angle;
  return Eigen::AngleAxisd(angle, axis);
}

const Eigen::Quaterniond RotationVector2Quaternion(const Eigen::Vector3d& v3d) {
  double n = v3d.norm();
  double n2 = n * n;
  double q0, q1, q2, q3;
  double s;
  if (n2 < 10.e-8) {
    q0 = 1.0 - n2 * (1.0 / 8.0 - (n2 / 384.0));
    s = 0.5 - n2 * (1.0 / 48.0 - n2 / 3840.0);
  } else {
    double n_2 = n / 2.0;
    q0 = cos(n_2);
    s = sin(n_2) / n;
  }
  return Eigen::Quaterniond(q0, s * v3d(0), s * v3d(1), s * v3d(2));
}

const Eigen::Matrix3d V3d2Skew(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d tmp;
  tmp << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;
  return tmp;
}

// rotation order is z(yaw), x(pitch), y(roll)
Eigen::Quaterniond Euler2Quaternion(const Eigen::Vector3d& euler) {
  double sp = sin(euler(0) / 2.0);
  double sr = sin(euler(1) / 2.0);
  double sy = sin(euler(2) / 2.0);
  double cp = cos(euler(0) / 2.0);
  double cr = cos(euler(1) / 2.0);
  double cy = cos(euler(2) / 2.0);
  return Eigen::Quaterniond(
      cp * cr * cy - sp * sr * sy, sp * cr * cy - cp * sr * sy,
      cp * sr * cy + sp * cr * sy, cp * cr * sy + sp * sr * cy);
}

Eigen::Vector3d Quaternion2Euler(const Eigen::Quaterniond& qua) {
  double q11 = qua.w() * qua.w();
  double q12 = qua.w() * qua.x();
  double q13 = qua.w() * qua.y();
  double q14 = qua.w() * qua.z();

  double q22 = qua.x() * qua.x();
  double q23 = qua.x() * qua.y();
  double q24 = qua.x() * qua.z();

  double q33 = qua.y() * qua.y();
  double q34 = qua.y() * qua.z();

  double q44 = qua.z() * qua.z();

  double c12 = 2.0 * (q23 - q14);
  double c22 = q11 - q22 + q33 - q44;
  double c31 = 2.0 * (q24 - q13);
  double c32 = 2.0 * (q34 + q12);
  double c33 = q11 - q22 - q33 + q44;

  return Eigen::Vector3d(asin(c32), atan2(-c31, c33), atan2(-c12, c22));
}