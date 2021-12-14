#include "motion_detect.h"

MotionDetect::MotionDetect() : motion_status_(kUnknow) {}

// 判断静止方法：
// 1，gyro的norm：sqrt(wx(i)*wx(i) + wy(i)*wy(i) + wz(i)*wz(i))
// 2，acce前后两帧的差：|ax(i) - ax(i-1)| + |ay(i) - ay(i-1)|
// 3，wz(i)*wz(i)
// 4，ay(i)*ay(i)
// 5，gyro和acce的std
void MotionDetect::Update(const IMUData& imu) {
  imu_buff_.emplace_back(imu);
  if (imu_buff_.size() > kImuBuffSize) {
    imu_buff_.pop_front();
  }
  if (imu_buff_.size() == kImuBuffSize) {
    GetGyroNorm();
    GetAcceDiff();
    GetImuMean();
    GetGyroStd();
    GetAcceStd();
    gyro_z2_ = pow(imu_buff_.back().Gyro()(2), 2);
    acce_y2_ = pow(imu_buff_.back().Acce()(1), 2);
    if (gyro_norm_ < kGyroNormThresh && acce_diff_ < kAcceDiffThresh &&
        gyro_z2_ < kGyroZThresh && acce_y2_ < kAcceYThresh &&
        gyro_std_(0) < kGyroStdThresh && gyro_std_(1) < kGyroStdThresh &&
        gyro_std_(2) < kGyroStdThresh && acce_std_(0) < kAcceStdThresh &&
        acce_std_(1) < kAcceStdThresh && acce_std_(2) < kAcceStdThresh)
      motion_status_ = kStatic;
  }
}

void MotionDetect::GetGyroNorm() {
  V3d gyro = imu_buff_.back().Gyro();
  gyro_norm_ = gyro.norm();
}

void MotionDetect::GetAcceDiff() {
  V3d acce = imu_buff_.back().Acce();
  auto it = imu_buff_.end();
  V3d acce_1 = std::prev(it)->Acce();
  acce_diff_ = fabsf(acce(0) - acce_1(0)) + fabsf(acce(1) - acce_1(1));
}

void MotionDetect::GetImuMean() {
  std::for_each(imu_buff_.begin(), imu_buff_.end(), [this](const IMUData& imu) {
    gyro_mean_ += imu.Gyro();
    acce_mean_ += imu.Acce();
  });
  gyro_mean_ /= imu_buff_.size();
  acce_mean_ /= imu_buff_.size();
}

void MotionDetect::GetGyroStd() {
  double x = 0.0, y = 0.0, z = 0.0;
  std::for_each(imu_buff_.begin(), imu_buff_.end(), [&](const IMUData& imu) {
    x += pow(imu.Gyro()(0) - gyro_mean_(0), 2);
    y += pow(imu.Gyro()(1) - gyro_mean_(1), 2);
    z += pow(imu.Gyro()(2) - gyro_mean_(2), 2);
  });
  x /= (imu_buff_.size() - 1);
  y /= (imu_buff_.size() - 1);
  z /= (imu_buff_.size() - 1);
  gyro_std_ = {x, y, z};
}

void MotionDetect::GetAcceStd() {
  double x = 0.0, y = 0.0, z = 0.0;
  std::for_each(imu_buff_.begin(), imu_buff_.end(), [&](const IMUData& imu) {
    x += pow(imu.Acce()(0) - acce_mean_(0), 2);
    y += pow(imu.Acce()(1) - acce_mean_(1), 2);
    z += pow(imu.Acce()(2) - acce_mean_(2), 2);
  });
  x /= (imu_buff_.size() - 1);
  y /= (imu_buff_.size() - 1);
  z /= (imu_buff_.size() - 1);
  acce_std_ = {x, y, z};
}