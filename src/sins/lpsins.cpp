// Copyright 2021 demax
#include "sins/lpsins.h"

#include <iostream>

namespace sins {

LPSINS::LPSINS() : SINS() {
  gn = V3d(0, 0, -9.8017);
  std::cout << "LPSINS construct!" << std::endl;
}

LPSINS::LPSINS(const V3d &att, const V3d &vel, const V3d &pos)
    : SINS(att, vel, pos) {
  gn = V3d(0, 0, -9.8017);
}

void LPSINS::Update(const IMUdata &imu) {
  UpdateAttitude(imu.Gyro());
  UpdateVelocity(imu.Acce());
  UpdatePosition();
}
void LPSINS::UpdateAttitude(const V3d &gyro) {
  double angle = gyro.norm();
  Eigen::Quaterniond qb(cos(angle / 2), gyro(0) * sin(angle / 2) / angle,
                        gyro(1) * sin(angle / 2) / angle,
                        gyro(2) * sin(angle / 2) / angle);
  q_ = q_ * qb;
}

void LPSINS::UpdateVelocity(const V3d &acce) {
  M3d Cnb(q_);
  velocity_ += (Cnb * acce + gn) * dt_;
}

void LPSINS::UpdatePosition() {
  position_ += (velocity_ + last_velocity_) * dt_ / 2;
}
}  // namespace sins
