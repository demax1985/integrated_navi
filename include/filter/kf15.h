// Copyright 2021 demax
#ifndef INCLUDE_FILTER_KF15_H_
#define INCLUDE_FILTER_KF15_H_

#include <memory>
#include <unsupported/Eigen/MatrixFunctions>

#include "filter/filter_base.h"
#include "sins/hpsins.h"
#include "sins/sins.h"

namespace sins {

class KF15 : public FilterBase {
 public:
  enum class KfErrorState {
    kPitch,
    kRoll,
    Kyaw,
    kVe,
    kVn,
    kVu,
    kLat,
    kLon,
    kAlt,
    kGbx,
    kGby,
    kGbz,
    kAbx,
    kAby,
    kAbz,
    kStateNumber
  };
  KF15() {}
  //    KF15(const Eigen::VectorXd& state, const Eigen::MatrixXd& P, const
  //    Eigen::MatrixXd& Q);
  KF15(const Eigen::Matrix<double, 15, 1> &state,
       const Eigen::Matrix<double, 15, 15> &P,
       const Eigen::Matrix<double, 15, 15> &Q,
       const std::shared_ptr<SINS> &sins, std::unique_ptr<SINS> sins_pre);
  void SetFk(double dt) override;
  void Predict(double dt) override;
  void MeasurementUpdate(const Eigen::VectorXd Zk, const Eigen::MatrixXd Hk,
                         const Eigen::MatrixXd Rk) override;
  void SetPkPositiveSymmetric() override;
  void CheckConvergence() override;

  void FeedbackAttitude() override;
  void FeedbackVelocity() override;
  void FeedbackPosition() override;
  void FeedbackGyroBias() override;
  void FeedbackAcceBias() override;
  void FeedbackAllState() override;

  int GetAttitudeIndex() override {
    return static_cast<int>(KfErrorState::kPitch);
  }
  int GetVnIndex() override { return static_cast<int>(KfErrorState::kVe); }
  int GetPosIndex() override { return static_cast<int>(KfErrorState::kLat); }
  int GetGyroBiasIndex() override {
    return static_cast<int>(KfErrorState::kGbx);
  }
  int GetAcceBiasIndex() override {
    return static_cast<int>(KfErrorState::kAbx);
  }

 private:
  std::shared_ptr<SINS> pSINS_;
  std::unique_ptr<SINS> pSINS_pre_;
};
}  // namespace sins
#endif  // INCLUDE_FILTER_KF15_H_
