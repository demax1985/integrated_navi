// Copyright 2021 demax
#ifndef INCLUDE_SINS_SINS_H_
#define INCLUDE_SINS_SINS_H_

#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

#include "common/global.h"
#include "sensors/imu.h"
#include "sins/earth.h"
namespace sins {

using V3d = Eigen::Vector3d;
using M3d = Eigen::Matrix3d;

class SINS {
 public:
  SINS();
  SINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts);
  virtual ~SINS() {}
  virtual void Update(const IMUData& imu) = 0;
  virtual void UpdateAttitude() = 0;
  virtual void UpdateVelocity() = 0;
  virtual void UpdatePosition() = 0;

  virtual void SetErrModelMatrix() = 0;

  virtual const M3d& Maa() const = 0;
  virtual const M3d& Mav() const = 0;
  virtual const M3d& Map() const = 0;
  virtual const M3d& Mva() const = 0;
  virtual const M3d& Mvv() const = 0;
  virtual const M3d& Mvp() const = 0;
  virtual const M3d& Mpv() const = 0;
  virtual const M3d& Mpp() const = 0;

  virtual const double TauG() const = 0;
  virtual const double TauA() const = 0;

  virtual void FeedbackAttitude(const V3d& phi) = 0;
  virtual void FeedbackVelocity(const V3d& dvn) = 0;
  virtual void FeedbackPosition(const V3d& dpos) = 0;
  virtual void FeedbackGyroBias(const V3d& gyro_bias) = 0;
  virtual void FeedbackAcceBias(const V3d& acce_bias) = 0;

  virtual void InitialLevelAlignment(const V3d& mean_acce_in_b_fram) = 0;

  const V3d& GetAttitude() const { return att_; }
  const V3d& GetVelocity() const { return vn_; }
  const V3d& GetPosition() const { return pos_; }
  const V3d& GetGyroBias() const { return gyro_bias_; }
  const V3d& GetAcceBias() const { return acce_bias_; }
  const M3d GetRotationMatrix() const { return q_.toRotationMatrix(); }

  void SetInitStatus(bool initialized) { initialized_ = initialized; }
  bool Initialized() { return initialized_; }
  double UpdateTimestamp() { return update_timestamp_; }
  void SetGyroBias(const V3d& gyrobias) { gyro_bias_ = gyrobias; }
  void SetAcceBias(const V3d& accebias) { acce_bias_ = accebias; }

 protected:
  bool initialized_;  // if initial alignment is completed
  V3d att_;           // pitch roll yaw
  V3d vn_, vn_prev_;
  V3d pos_;
  V3d an_;
  Eigen::Quaterniond q_, q_prev_;
  double update_timestamp_, pre_update_timestamp_, dt_;
  // ts_: imu data interval
  double current_imu_timestamp_, prev_imu_timestamp_, ts_;

  V3d gyro_bias_, acce_bias_;

  // error model coefficient
  M3d Maa_, Mav_, Map_, Mva_, Mvv_, Mvp_, Mpv_, Mpp_;
};
}  // namespace sins
#endif  // INCLUDE_SINS_SINS_H_
