// Copyright 2021 demax
#ifndef INCLUDE_MOTION_DETECT_H
#define INCLUDE_MOTION_DETECT_H
#include <Eigen/Core>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>

#include "sensors/imu.h"

using V3d = Eigen::Vector3d;

class MotionDetect {
 private:
  enum MotionStatus { kUnknow, kStatic, kUniform, kAccelerate, kDecelerate };
  std::ofstream outfile_;
  // some const values
  // TODO(demax): need to post check the constants
  static const int kImuBuffSize = 100;
  static constexpr double kGyroNormThresh = 0.01;
  static constexpr double kAcceDiffThresh = 0.01;
  static constexpr double kGyroZThresh = 0.01;
  static constexpr double kAcceYThresh = 0.01;
  static constexpr double kGyroStdThresh = 0.01;
  static constexpr double kAcceStdThresh = 0.01;

  std::deque<IMUData> imu_buff_;
  MotionStatus motion_status_;
  // some statistics for motion detect
  double gyro_norm_;
  double acce_diff_;
  double gyro_z_;
  double acce_y_;
  V3d gyro_std_, acce_std_;

  V3d gyro_mean_, acce_mean_;

  void GetGyroNorm();
  void GetAcceDiff();
  void GetImuMean();
  void GetGyroStd();
  void GetAcceStd();

 public:
  MotionDetect();
  ~MotionDetect();
  void Update(const IMUData& imu);
  MotionStatus GetMotionStatus() const { return motion_status_; }
};

#endif
