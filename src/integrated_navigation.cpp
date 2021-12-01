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
  mean_gyro_static_.setZero();
  imu_sub_ =
      nh_.subscribe("/IMU_data", 1, &IntegratedNavigation::ImuCallback, this);
  gnss_sub_ =
      nh_.subscribe("/gnss_data", 1, &IntegratedNavigation::GnssCallback, this);
  outfile_.open("result.txt");
  outfile_ << "pitch, roll, yaw, ve, vn, vu, lat, lon, alt, gyrobias, accebias"
           << std::endl;
}

IntegratedNavigation::IntegratedNavigation(std::shared_ptr<SINS> sins,
                                           std::unique_ptr<FilterBase> filter)
    : is_static_(true),
      gnss_vel_valid_(false),
      gnss_pos_valid_(false),
      baro_alt_(0.0),
      gnss_yaw_(0.0),
      zihr_initial_yaw_(0.0),
      zihr_initial_time_(0.0),
      initial_alignment_count_(0),
      kf_predict_dt_(0.0),
      kf_predict_time_prev_(0.0),
      pFilter_(std::move(filter)) {
  mean_acce_in_b_fram_.setZero();
  mean_gyro_static_.setZero();
  imu_sub_ =
      nh_.subscribe("/IMU_data", 1, &IntegratedNavigation::ImuCallback, this);
  gnss_sub_ =
      nh_.subscribe("/gnss_data", 1, &IntegratedNavigation::GnssCallback, this);
  outfile_.open("result.txt");
  outfile_ << "pitch, roll, yaw, ve, vn, vu, lat, lon, alt, gyrobias, accebias"
           << std::endl;
  pSINS_ = sins;
  std::cout << "IntegratedNavigation constructed, sins pos is: "
            << pSINS_->GetPosition() << std::endl;
  //   pFilter_ = std::move(filter);
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
      mean_gyro_static_ += gyro;
      if (initial_alignment_count_++ == kInitialAlignmentCount) {
        mean_acce_in_b_fram_ /= initial_alignment_count_;
        mean_gyro_static_ /= initial_alignment_count_;
        pSINS_->SetGyroBias(mean_gyro_static_);
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
  outfile_ << pSINS_->GetAttitude()(0) << "  " << pSINS_->GetAttitude()(1)
           << "  " << pSINS_->GetAttitude()(2) << "  "
           << pSINS_->GetVelocity()(0) << "  " << pSINS_->GetVelocity()(1)
           << "  " << pSINS_->GetVelocity()(2) << "  "
           << pSINS_->GetPosition()(0) << "  " << pSINS_->GetPosition()(1)
           << "  " << pSINS_->GetPosition()(2) << "  "
           << pSINS_->GetGyroBias()(0) << "  " << pSINS_->GetGyroBias()(1)
           << "  " << pSINS_->GetGyroBias()(2) << "  "
           << pSINS_->GetAcceBias()(0) << "  " << pSINS_->GetAcceBias()(1)
           << "  " << pSINS_->GetAcceBias()(2) << std::endl;
}
void IntegratedNavigation::GnssCallback(
    const sensor_msgs::NavSatFixConstPtr& gnss_pos) {
  if (!pSINS_->Initialized()) {
    return;
  }
  gnss_.gnss_pos_ = {gnss_pos->latitude, gnss_pos->longitude,
                     gnss_pos->altitude};
  double timestamp = gnss_pos->header.stamp.toSec();
  double dt = timestamp - kf_predict_time_prev_;
  V3d tmpRk = {gnss_pos->position_covariance[0],
               gnss_pos->position_covariance[4],
               gnss_pos->position_covariance[8]};
  M3d Rk = tmpRk.asDiagonal();

  std::cout << " gnss pose is: " << gnss_.gnss_pos_ << std::endl;
  std::cout << " sins pose is: " << pSINS_->GetPosition() << std::endl;

  int state_num = pFilter_->GetStateNumber();
  int pos_index = pFilter_->GetPosIndex();

  // std::cout << "state num is: " << state_num << ", pos_index is: " <<
  // pos_index
  //           << std::endl;
  Eigen::MatrixXd Hk;
  Hk.resize(3, state_num);

  Hk.block<3, 3>(0, pos_index) = Eigen::Matrix3d::Identity();
  std::cout << "Hk is: " << Hk << std::endl;
  std::cout << "dt of gnss is: " << dt << std::endl;
  if (dt > 0) {
    pFilter_->Predict(dt);
    kf_predict_time_prev_ = timestamp;
    V3d Zk = pSINS_->GetPosition() - gnss_.gnss_pos_;
    pFilter_->MeasurementUpdate(Zk, Hk, Rk);
    pFilter_->FeedbackAllState();
  } else {
    dt *= -1;
    V3d gnss_pos_extrapolated =
        gnss_.gnss_pos_ + pSINS_->Mpv() * pSINS_->GetVelocity() * dt;
    V3d Zk = pSINS_->GetPosition() - gnss_pos_extrapolated;
    pFilter_->MeasurementUpdate(Zk, Hk, Rk);
    pFilter_->FeedbackAllState();
  }
}