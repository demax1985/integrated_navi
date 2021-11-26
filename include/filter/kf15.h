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
    kAbz
  };
  KF15() {}
  //    KF15(const Eigen::VectorXd& state, const Eigen::MatrixXd& P, const
  //    Eigen::MatrixXd& Q);
  KF15(const Eigen::Matrix<double, 15, 1> &state,
       const Eigen::Matrix<double, 15, 15> &P,
       const Eigen::Matrix<double, 15, 15> &Q, std::shared_ptr<SINS> sins);
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

 private:
  std::shared_ptr<SINS> pSINS_;
};
}  // namespace sins
#endif  // INCLUDE_FILTER_KF15_H_
