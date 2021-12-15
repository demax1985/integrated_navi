#include "motion_detect.h"

MotionDetect::MotionDetect()
    : motion_status_(kUnknow),
      gyro_norm_(0.0),
      acce_diff_(0.0),
      gyro_z_(0.0),
      acce_y_(0.0) {
  gyro_std_.setZero();
  acce_std_.setZero();
  gyro_mean_.setZero();
  acce_mean_.setZero();
  outfile_.open("motion_detect.txt");
  outfile_ << "gyro_norm"
           << "  "
           << "acce_diff"
           << "  "
           << "gyro_z2"
           << "  "
           << "acce_y2"
           << "  "
           << "gyro_stdx"
           << "  "
           << "gyro_stdy"
           << "  "
           << "gyro_stdz"
           << "  "
           << "acce_stdx"
           << "  "
           << "acce_stdy"
           << "  "
           << "acce_stdz" << std::endl;
}
MotionDetect::~MotionDetect() { outfile_.close(); }

// 判断静止方法：
// 1，gyro的norm：sqrt(wx(i)*wx(i) + wy(i)*wy(i) + wz(i)*wz(i))
// 2，acce前后两帧的差：|ax(i) - ax(i-1)| + |ay(i) - ay(i-1)|
// 3，abs(wz(i))
// 4，abs(ay(i))
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
    gyro_z_ = fabs(imu_buff_.back().Gyro()(2));
    acce_y_ = fabs(imu_buff_.back().Acce()(1));
    if (gyro_norm_ < kGyroNormThresh && acce_diff_ < kAcceDiffThresh &&
        gyro_z_ < kGyroZThresh && acce_y_ < kAcceYThresh &&
        gyro_std_(0) < kGyroStdThresh && gyro_std_(1) < kGyroStdThresh &&
        gyro_std_(2) < kGyroStdThresh && acce_std_(0) < kAcceStdThresh &&
        acce_std_(1) < kAcceStdThresh && acce_std_(2) < kAcceStdThresh) {
      motion_status_ = kStatic;
    }
    outfile_ << gyro_norm_ << "  " << acce_diff_ << "  " << gyro_z_ << "  "
             << acce_y_ << "  " << gyro_std_(0) << "  " << gyro_std_(1) << "  "
             << gyro_std_(2) << "  " << acce_std_(0) << "  " << acce_std_(1)
             << "  " << acce_std_(2) << "  " << std::endl;
  }
}

void MotionDetect::GetGyroNorm() {
  V3d gyro = imu_buff_.back().Gyro();
  gyro_norm_ = gyro.norm();
}

void MotionDetect::GetAcceDiff() {
  V3d acce = imu_buff_.back().Acce();
  auto it = imu_buff_.rbegin();
  V3d acce_1 = std::prev(it)->Acce();
  acce_diff_ = fabsf(acce(0) - acce_1(0)) + fabsf(acce(1) - acce_1(1));
}

void MotionDetect::GetImuMean() {
  gyro_mean_.setZero();
  acce_mean_.setZero();
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
  x = sqrt(x);
  y = sqrt(y);
  z = sqrt(z);
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
  x = sqrt(x);
  y = sqrt(y);
  z = sqrt(z);
  acce_std_ = {x, y, z};
}
