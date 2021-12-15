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
      kf_predict_time_prev_(0.0),
      gnss_fusion_count_(0),
      static_count_(0),
      pFilter_(new sins::KF15()),
      pSINS_(new sins::LPSINS()),
      pMotionDetect_(new MotionDetect()) {
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

IntegratedNavigation::IntegratedNavigation(const std::shared_ptr<SINS>& sins,
                                           std::unique_ptr<FilterBase> filter)
    : is_static_(false),
      gnss_vel_valid_(false),
      gnss_pos_valid_(false),
      baro_alt_(0.0),
      gnss_yaw_(0.0),
      zihr_initial_yaw_(0.0),
      zihr_initial_time_(0.0),
      initial_alignment_count_(0),
      kf_predict_dt_(0.0),
      kf_predict_time_prev_(0.0),
      gnss_fusion_count_(0),
      static_count_(0),
      pFilter_(std::move(filter)),
      pSINS_(sins),
      pMotionDetect_(new MotionDetect()) {
  mean_acce_in_b_fram_.setZero();
  mean_gyro_static_.setZero();
  imu_sub_ =
      nh_.subscribe("/IMU_data", 1, &IntegratedNavigation::ImuCallback, this);
  gnss_sub_ =
      nh_.subscribe("/gnss_data", 1, &IntegratedNavigation::GnssCallback, this);
  outfile_.open("result.txt");
  outfile_ << "pitch, roll, yaw, ve, vn, vu, lat, lon, alt, gyrobias, accebias"
           << std::endl;
  Eigen::Quaterniond qua;
  qua = pSINS_->GetQuaternion();
  std::cout << "qua of integrated navigation construct is: " << std::endl;
  std::cout << qua.w() << "  " << qua.x() << "  " << qua.y() << "  " << qua.z()
            << std::endl;
  std::cout << "cnb is: " << std::endl;
  std::cout << pSINS_->GetRotationMatrix() << std::endl;
  std::cout << "IntegratedNavigation constructed, sins pos is: "
            << pSINS_->GetPosition() << std::endl;
  //   pFilter_ = std::move(filter);
}

void IntegratedNavigation::ImuCallback(const sensor_msgs::ImuConstPtr& imu) {
  V3d gyro = {imu->angular_velocity.x, imu->angular_velocity.y,
              imu->angular_velocity.z};
  V3d acce = {imu->linear_acceleration.x, imu->linear_acceleration.y,
              imu->linear_acceleration.z};
  gyro -= pSINS_->GetGyroBias();
  acce -= pSINS_->GetAcceBias();
  double timestamp = imu->header.stamp.toSec();
  imu_ = IMUData(gyro, acce, timestamp);
  pMotionDetect_->Update(imu_);
  is_static_ = (1 == static_cast<int>(pMotionDetect_->GetMotionStatus()));
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
    if (pSINS_->GetSINSUpdateCount()) {
      pFilter_->Predict(kf_predict_dt_);
    }
    // std::cout << "kf_predict_dt_ is: " << kf_predict_dt_ << std::endl;
    kf_predict_time_prev_ = pSINS_->UpdateTimestamp();
  }
}
void IntegratedNavigation::GnssCallback(
    const sensor_msgs::NavSatFixConstPtr& gnss_pos) {
  if (!pSINS_->Initialized() || pSINS_->GetSINSUpdateCount() < 1) {
    return;
  }
  gnss_fusion_count_++;
  std::cout << "sins update count is: " << pSINS_->GetSINSUpdateCount()
            << std::endl;
  std::cout << "================" << gnss_fusion_count_
            << "===============" << std::endl;
  std::cout << "sins att is: " << std::endl;
  std::cout << pSINS_->GetAttitude() << std::endl;
  std::cout << "sins vn is: " << std::endl;
  std::cout << pSINS_->GetVelocity() << std::endl;
  std::cout << "sins pos is: " << std::endl;
  std::cout << pSINS_->GetPosition() << std::endl;
  gnss_.gnss_pos_ = {gnss_pos->latitude, gnss_pos->longitude,
                     gnss_pos->altitude};
  double timestamp = gnss_pos->header.stamp.toSec();
  double dt = timestamp - kf_predict_time_prev_;
  V3d tmpRk = {gnss_pos->position_covariance[0],
               gnss_pos->position_covariance[4],
               gnss_pos->position_covariance[8]};
  M3d Rk = tmpRk.asDiagonal();
  // std::cout.precision(10);
  // std::cout << " gnss pose is: " << gnss_.gnss_pos_ << std::endl;
  // std::cout << " sins pose is: " << pSINS_->GetPosition() << std::endl;
  // std::cout << " dt is: " << dt << std::endl;

  int state_num = pFilter_->GetStateNumber();
  int pos_index = pFilter_->GetPosIndex();

  // std::cout << "state num is: " << state_num << ", pos_index is: " <<
  // pos_index
  //           << std::endl;
  Eigen::MatrixXd Hk;
  Hk.resize(3, state_num);
  Hk.setZero();
  Hk.block<3, 3>(0, pos_index) = Eigen::Matrix3d::Identity();
  std::cout << "Hk is: " << std::endl;
  std::cout << Hk << std::endl;
  std::cout << "Rk is: " << std::endl;
  std::cout << Rk << std::endl;
  // std::cout << "dt of gnss is: " << dt << std::endl;

  V3d gnss_pos_extrapolated =
      gnss_.gnss_pos_ - pSINS_->Mpv() * pSINS_->GetVelocity() * dt;
  V3d Zk = pSINS_->GetPosition() - gnss_.gnss_pos_;
  std::cout << " observation is: " << std::endl;
  std::cout << Zk(0) * 6378137 << "  " << Zk(1) * 6378137 << "  " << Zk(2)
            << std::endl;
  pFilter_->MeasurementUpdate(Zk, Hk, Rk);
  pFilter_->FeedbackAllState();
  // pFilter_->FeedbackAttitude();
  // pFilter_->FeedbackVelocity();
  // pFilter_->FeedbackPosition();

  outfile_ << pSINS_->GetAttitude()(0) << "  " << pSINS_->GetAttitude()(1)
           << "  " << pSINS_->GetAttitude()(2) << "  "
           << pSINS_->GetVelocity()(0) << "  " << pSINS_->GetVelocity()(1)
           << "  " << pSINS_->GetVelocity()(2) << "  "
           << pSINS_->GetPosition()(0) << "  " << pSINS_->GetPosition()(1)
           << "  " << pSINS_->GetPosition()(2) << "  "
           << pSINS_->GetGyroBias()(0) * 57.3 * 3600 << "  "
           << pSINS_->GetGyroBias()(1) * 57.3 * 3600 << "  "
           << pSINS_->GetGyroBias()(2) * 57.3 * 3600 << "  "
           << pSINS_->GetAcceBias()(0) * 100 << "  "
           << pSINS_->GetAcceBias()(1) * 100 << "  "
           << pSINS_->GetAcceBias()(2) * 100 << "  " << std::endl;
  //  << pFilter_->GetState()(9) * 57.3 * 3600 << "  "
  //  << pFilter_->GetState()(10) * 57.3 * 3600 << "  "
  //  << pFilter_->GetState()(11) * 57.3 * 3600 << "  "
  //  << pFilter_->GetState()(12) * 100 << "  "
  //  << pFilter_->GetState()(13) * 100 << "  "
  //  << pFilter_->GetState()(14) * 100 << std::endl;
}
