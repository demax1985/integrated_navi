// Copyright 2021 demax
#include "integrated_navigation.h"

IntegratedNavigation::IntegratedNavigation()
    : is_static_(false),
      gnss_vel_valid_(false),
      gnss_pos_valid_(false),
      baro_alt_(0.0),
      gnss_yaw_(0.0),
      zihr_initial_yaw_(0.0),
      zihr_initial_time_(0.0),
      initial_alignment_count_(0),
      kf_predict_dt_(0.0),
      kf_predict_time_prev_(0.0) {
  mean_acce_in_b_fram_.setZero();
  imu_sub_ =
      nh_.subscribe("/IMU_data", 1, &IntegratedNavigation::ImuCallback, this);
  gnss_sub_ =
      nh_.subscribe("/gnss_data", 1, &IntegratedNavigation::GnssCallback, this);
  outfile_.open("result.txt");
  outfile_ << "pitch, roll, yaw, ve, vn, vu, lat, lon, alt, gyrobias, accebias"
           << std::endl;
}

IntegratedNavigation::IntegratedNavigation(std::shared_ptr<SINS> sins,
                                           std::unique_ptr<FilterBase> filter,
                                           FusionAlgorithm algo)
    : is_static_(false),
      gnss_vel_valid_(false),
      gnss_pos_valid_(false),
      baro_alt_(0.0),
      gnss_yaw_(0.0),
      zihr_initial_yaw_(0.0),
      zihr_initial_time_(0.0),
      initial_alignment_count_(0),
      kf_predict_dt_(0.0),
      kf_predict_time_prev_(0.0) {
  mean_acce_in_b_fram_.setZero();
  imu_sub_ =
      nh_.subscribe("/IMU_data", 1, &IntegratedNavigation::ImuCallback, this);
  gnss_sub_ =
      nh_.subscribe("/gnss_data", 1, &IntegratedNavigation::GnssCallback, this);
  outfile_.open("result.txt");
  outfile_ << "pitch, roll, yaw, ve, vn, vu, lat, lon, alt, gyrobias, accebias"
           << std::endl;
  if (algo == kFilter) {
    pSINS_ = sins;
    pFilter_ = std::move(filter);
  } else {
    // TODO(demax): FactorGraphOptimization implimentation
  }
}

void IntegratedNavigation::ImuCallback(const sensor_msgs::ImuConstPtr& imu) {
  V3d gyro = {imu->angular_velocity.x, imu->angular_velocity.y,
              imu->angular_velocity.z};
  V3d acce = {imu->linear_acceleration.x, imu->linear_acceleration.y,
              imu->linear_acceleration.z};
  double timestamp = imu->header.stamp.toSec();
  imu_ = IMUData(gyro, acce, timestamp);
  if (!pSINS_->Initialized()) {
    if (is_static_) {
      mean_acce_in_b_fram_ += acce;
      if (initial_alignment_count_++ == kInitialAlignmentCount) {
        mean_acce_in_b_fram_ /= initial_alignment_count_;
        pSINS_->InitialLevelAlignment(mean_acce_in_b_fram_);
        pSINS_->SetInitStatus(true);
        kf_predict_time_prev_ = timestamp;
        std::cout << "initial level alignment completed!" << std::endl;
      }
    } else {
      initial_alignment_count_ = 0;
      mean_acce_in_b_fram_.setZero();
    }
  } else {
    pSINS_->Update(imu_);
    kf_predict_dt_ = pSINS_->UpdateTimestamp() - kf_predict_time_prev_;
    if (kf_predict_dt_ > kKfPredictDt) {
      pFilter_->Predict(kf_predict_dt_);
      kf_predict_time_prev_ = pSINS_->UpdateTimestamp();
    }
  }
  outfile_ << pSINS_->GetAttitude() << "  " << pSINS_->GetVelocity() << "  "
           << pSINS_->GetPosition() << "  " << pSINS_->GetGyroBias() << "  "
           << pSINS_->GetAcceBias() << std::endl;
}
void IntegratedNavigation::GnssCallback(
    const sensor_msgs::NavSatFixConstPtr& gnss_pos) {
  gnss_.gnss_pos_ = {gnss_pos->latitude, gnss_pos->longitude,
                     gnss_pos->altitude};
  double timestamp = gnss_pos->header.stamp.toSec();
  double dt = timestamp - kf_predict_time_prev_;
  V3d Rk = {gnss_pos->position_covariance[0], gnss_pos->position_covariance[4],
            gnss_pos->position_covariance[8]};
  int state_num = pFilter_->GetStateNumber();
  int pos_index = pFilter_->GetPosIndex();
  Eigen::MatrixXd Hk;
  Hk.resize(state_num, 3);
  Hk.block<3, 3>(0, pos_index) = Eigen::Matrix3d::Identity();
  if (dt > 0) {
    pFilter_->Predict(dt);
    kf_predict_time_prev_ = timestamp;
    V3d Zk = pSINS_->GetPosition() - gnss_.gnss_pos_;
    pFilter_->MeasurementUpdate(Zk, Hk, Rk);
    pFilter_->FeedbackAllState();
  } else {
    dt *= -1;
    V3d gnss_pos_extrapolated =
        gnss_.gnss_pos_ + pSINS_->GetVelocity() * pSINS_->Mpv() * dt;
    V3d Zk = pSINS_->GetPosition() - gnss_pos_extrapolated;
    pFilter_->MeasurementUpdate(Zk, Hk, Rk);
    pFilter_->FeedbackAllState();
  }
}