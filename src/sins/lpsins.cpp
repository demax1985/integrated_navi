// Copyright 2021 demax
#include "sins/lpsins.h"

#include <iostream>

namespace sins {

LPSINS::LPSINS() : SINS() {
  gn = V3d(0, 0, -9.8017);
  eth_ = std::unique_ptr<Earth>(new Earth());
  std::cout << "LPSINS construct!" << std::endl;
}

LPSINS::LPSINS(const V3d& att, const V3d& vel, const V3d& pos, double ts,
               double taug, double taua)
    : SINS(att, vel, pos, ts, taua, taua) {
  gn = V3d(0, 0, -9.8017);
  eth_ = std::unique_ptr<Earth>(new Earth(pos, vel));
}

LPSINS::LPSINS(const LPSINS& other) : SINS(other) {
  gn = other.gn;
  eth_ = std::unique_ptr<Earth>(new Earth(*other.eth_));
}

void LPSINS::Update(const IMUData& imu) {
  current_imu_timestamp_ = imu.Timestamp();
  update_timestamp_ = current_imu_timestamp_;
  dt_ = update_timestamp_ - pre_update_timestamp_;
  if (dt_ > 0.1) {
    pre_update_timestamp_ = update_timestamp_;
    return;
  }
  wib_ = imu.Gyro();
  fb_ = imu.Acce();
  UpdateAttitude();
  UpdateVelocity();
  UpdatePosition();
  pre_update_timestamp_ = update_timestamp_;
  vn_prev_ = vn_;
  sins_update_count_++;
}
void LPSINS::UpdateAttitude() {
  Eigen::Quaterniond q_b_ib = RotationVector2Quaternion(wib_ * dt_);
  q_ = q_ * q_b_ib;
  q_.normalize();
  att_ = Quaternion2Euler(q_);
}

void LPSINS::UpdateVelocity() {
  an_ = q_ * fb_ + EarthGcc();
  vn_ += an_ * dt_;
}

void LPSINS::UpdatePosition() {
  pos_ += Vn2DeltaPos((vn_ + vn_prev_) / 2.0, dt_);
}

void LPSINS::SetErrModelMatrix() {
  Maa_.setZero();
  Mav_.setZero();
  Map_.setZero();
  V3d fn = q_ * fb_;
  Mva_ = V3d2Skew(fn);
  Mvv_.setZero();
  Mvp_.setZero();
  double f_Rmh = 1.0 / EarthRmh();
  double f_clRnh = 1.0 / EarthClRnh();
  Mpv_(0, 1) = f_Rmh;
  Mpv_(1, 0) = f_clRnh;
  Mpv_(2, 2) = 1.0;
  Mpp_.setZero();
}
double LPSINS::EarthRmh() const { return eth_->Rmh(); }
double LPSINS::EarthRnh() const { return eth_->Rnh(); }
double LPSINS::EarthClRnh() const { return eth_->Rnh(); }
const V3d LPSINS::Vn2DeltaPos(const V3d& vn, double dt) const {
  return V3d(vn(1) * dt / EarthRmh(), vn(0) * dt / EarthClRnh(), vn(2) * dt);
}

const V3d& LPSINS::EarthGcc() const { return eth_->Gcc(); }

const M3d& LPSINS::Maa() const { return Maa_; }
const M3d& LPSINS::Mav() const { return Mav_; }
const M3d& LPSINS::Map() const { return Map_; }
const M3d& LPSINS::Mva() const { return Mva_; }
const M3d& LPSINS::Mvv() const { return Mvv_; }
const M3d& LPSINS::Mvp() const { return Mvp_; }
const M3d& LPSINS::Mpv() const { return Mpv_; }
const M3d& LPSINS::Mpp() const { return Mpp_; }

void LPSINS::FeedbackAttitude(const V3d& phi) {
  q_ = RotationVector2Quaternion(phi) * q_;
}
void LPSINS::FeedbackVelocity(const V3d& dvn) { vn_ -= dvn; }
void LPSINS::FeedbackPosition(const V3d& dpos) { pos_ -= dpos; }
void LPSINS::FeedbackGyroBias(const V3d& gyro_bias) { gyro_bias_ += gyro_bias; }
void LPSINS::FeedbackAcceBias(const V3d& acce_bias) { acce_bias_ += acce_bias; }

const double LPSINS::TauG() const { return tauG_; }

const double LPSINS::TauA() const { return tauA_; }

void LPSINS::InitialLevelAlignment(const V3d& mean_acce_in_b_fram) {
  // double pitch = asin(mean_acce_in_b_fram(1) / EarthG0());
  // double roll = atan2(-mean_acce_in_b_fram(0), mean_acce_in_b_fram(2));
  // att_ = {pitch, roll, 0};
  // q_ = Euler2Quaternion(att_);
  // q_prev_ = q_;
  fb_ = mean_acce_in_b_fram;
  SetInitStatus(true);
}
}  // namespace sins
