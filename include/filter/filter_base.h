// Copyright 2021 demax
#ifndef INCLUDE_FILTER_FILTER_BASE_H_
#define INCLUDE_FILTER_FILTER_BASE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class FilterBase {
 public:
  FilterBase() {}
  FilterBase(const Eigen::VectorXd& state, const Eigen::MatrixXd& P,
             const Eigen::MatrixXd& Q);
  virtual ~FilterBase() {}
  virtual void SetFk(double dt) = 0;
  virtual void Predict(double dt) = 0;
  virtual void MeasurementUpdate(const Eigen::VectorXd Zk,
                                 const Eigen::MatrixXd Hk,
                                 const Eigen::MatrixXd Rk) = 0;
  virtual void SetPkPositiveSymmetric() = 0;
  virtual void CheckConvergence() = 0;

  virtual void FeedbackAttitude() { return; }
  virtual void FeedbackVelocity() { return; }
  virtual void FeedbackPosition() { return; }
  virtual void FeedbackGyroBias() { return; }
  virtual void FeedbackAcceBias() { return; }
  virtual void FeedbackAllState() { return; }

  virtual int GetAttitudeIndex() { return 0; }
  virtual int GetVnIndex() { return 0; }
  virtual int GetPosIndex() { return 0; }
  virtual int GetGyroBiasIndex() { return 0; }
  virtual int GetAcceBiasIndex() { return 0; }

  int GetStateNumber() { return state_.rows(); }
  const Eigen::VectorXd& GetState() const { return state_; }

 protected:
  bool converged_;
  double dt_;
  Eigen::VectorXd state_;
  Eigen::MatrixXd Pk_;
  Eigen::MatrixXd Qt_;
  Eigen::MatrixXd Fk_;
};

#endif  // INCLUDE_FILTER_FILTER_BASE_H_
