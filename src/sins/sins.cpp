// Copyright 2021 demax
#include "sins/sins.h"

namespace sins {

SINS::SINS()
    : update_timestamp_(0),
      pre_update_timestamp_(0),
      dt_(0),
      ts_(0.01),
      current_imu_timestamp_(0.0),
      prev_imu_timestamp_(0.0),
      initialized_(false) {
  att_.setZero();
  vn_.setZero();
  vn_prev_.setZero();
  an_.setZero();
  pos_.setZero();
  q_.setIdentity();
  q_prev_.setIdentity();
  gyro_bias_.setZero();
  acce_bias_.setZero();
  Maa_.setZero();
  Mav_.setZero();
  Map_.setZero();
  Mva_.setZero();
  Mvv_.setZero();
  Mvp_.setZero();
  Mpv_.setZero();
  Mpp_.setZero();
}

SINS::SINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts)
    : att_(att),
      vn_(vn),
      vn_prev_(vn),
      pos_(pos),
      update_timestamp_(0),
      pre_update_timestamp_(0),
      dt_(0),
      ts_(ts),
      current_imu_timestamp_(0.0),
      prev_imu_timestamp_(0.0),
      initialized_(false) {
  q_ = Euler2Quaternion(att_);
  q_prev_ = q_;
  gyro_bias_.setZero();
  acce_bias_.setZero();
  an_.setZero();
  Maa_.setZero();
  Mav_.setZero();
  Map_.setZero();
  Mva_.setZero();
  Mvv_.setZero();
  Mvp_.setZero();
  Mpv_.setZero();
  Mpp_.setZero();
  std::cout << "sins constructed, pos is: " << pos_ << std::endl;
}

}  // namespace sins
