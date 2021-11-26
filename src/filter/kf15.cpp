// Copyright 2021 demax
#include "filter/kf15.h"
namespace sins {

KF15::KF15(const Eigen::Matrix<double, 15, 1> &state,
           const Eigen::Matrix<double, 15, 15> &P,
           const Eigen::Matrix<double, 15, 15> &Q, std::shared_ptr<SINS> sins)
    : FilterBase(state, P, Q) {
  pSINS_ = sins;
  Fk_.resize(15, 15);
  Fk_.setZero();
}

void KF15::Predict(double dt) {
  SetFk(dt);
  state_ = Fk_ * state_;
  Pk_ = Fk_ * Pk_ * Fk_.transpose() + Qt_ * dt;
}

void KF15::MeasurementUpdate(const Eigen::VectorXd Zk, const Eigen::MatrixXd Hk,
                             const Eigen::MatrixXd Rk) {
  auto Pxy = Pk_ * Hk.transpose();
  auto Py0 = Hk * Pxy;
  auto Py = Py0 + Rk;
  auto innovation = Zk - Hk * state_;
  auto Kk = Pxy * Py.inverse();
  std::cout << "Kk is: " << std::endl << Kk << std::endl;
  state_ += Kk * innovation;
  Pk_ -= Kk * Hk * Pk_;
  SetPkPositiveSymmetric();
  std::cout << "after measurement update, state is: " << std::endl
            << state_ << std::endl;
}

void KF15::SetFk(double dt) {
  pSINS_->SetErrModelMatrix();
  Eigen::Matrix<double, 15, 15> Ft;
  Ft.setZero();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kPitch),
                 static_cast<int>(KfErrorState::kPitch)) = pSINS_->Maa();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kPitch),
                 static_cast<int>(KfErrorState::kVe)) = pSINS_->Mav();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kPitch),
                 static_cast<int>(KfErrorState::kLat)) = pSINS_->Map();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kPitch),
                 static_cast<int>(KfErrorState::kGbx)) =
      -pSINS_->GetRotationMatrix();

  Ft.block<3, 3>(static_cast<int>(KfErrorState::kVe),
                 static_cast<int>(KfErrorState::kPitch)) = pSINS_->Mva();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kVe),
                 static_cast<int>(KfErrorState::kVe)) = pSINS_->Mvv();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kVe),
                 static_cast<int>(KfErrorState::kLat)) = pSINS_->Mvp();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kVe),
                 static_cast<int>(KfErrorState::kAbx)) =
      pSINS_->GetRotationMatrix();

  Ft.block<3, 3>(static_cast<int>(KfErrorState::kLat),
                 static_cast<int>(KfErrorState::kVe)) = pSINS_->Mpv();
  Ft.block<3, 3>(static_cast<int>(KfErrorState::kLat),
                 static_cast<int>(KfErrorState::kLat)) = pSINS_->Mpp();

  Eigen::Matrix<double, 6, 1> tau;
  const double tauG = pSINS_->TauG();
  const double tauA = pSINS_->TauA();
  tau << -1.0 / tauG, -1.0 / tauG, -1.0 / tauG, -1.0 / tauA, -1.0 / tauA,
      -1.0 / tauA;
  Eigen::DiagonalMatrix<double, 6> tauGA = tau.asDiagonal();
  Ft.block<6, 6>(static_cast<int>(KfErrorState::kGbx),
                 static_cast<int>(KfErrorState::kGbx)) = tauGA;
  Ft *= dt;
  Fk_ = Ft.exp();
  std::cout << "Fk is: " << std::endl << Fk_ << std::endl;
}

void KF15::SetPkPositiveSymmetric() {
  for (int i = 0; i < Pk_.rows(); i++)
    for (int j = 0; j < Pk_.cols(); j++) {
      if (Pk_(i, j) < 0) {
        Pk_(i, j) = -Pk_(i, j);
      }
    }
  // TODO(demax): the following code maybe wrong
  Pk_ = (Pk_ + Pk_.transpose()) / 2.0;
  std::cout << "Pk is: " << std::endl << Pk_ << std::endl;
}

// TODO(demax): coding later
void KF15::CheckConvergence() { ; }

void KF15::FeedbackAllState() {
  FeedbackAttitude();
  FeedbackVelocity();
  FeedbackPosition();
  FeedbackGyroBias();
  FeedbackAcceBias();
}

void KF15::FeedbackAttitude() {
  V3d phi = {state_(static_cast<int>(KfErrorState::kPitch)),
             state_(static_cast<int>(KfErrorState::kRoll)),
             state_(static_cast<int>(KfErrorState::Kyaw))};
  pSINS_->FeedbackAttitude(phi);
  state_(static_cast<int>(KfErrorState::kPitch)) = 0;
  state_(static_cast<int>(KfErrorState::kRoll)) = 0;
  state_(static_cast<int>(KfErrorState::Kyaw)) = 0;
}
void KF15::FeedbackVelocity() {
  V3d dvn = {state_(static_cast<int>(KfErrorState::kVe)),
             state_(static_cast<int>(KfErrorState::kVn)),
             state_(static_cast<int>(KfErrorState::kVu))};
  pSINS_->FeedbackVelocity(dvn);
  state_(static_cast<int>(KfErrorState::kVe)) = 0;
  state_(static_cast<int>(KfErrorState::kVn)) = 0;
  state_(static_cast<int>(KfErrorState::kVu)) = 0;
}
void KF15::FeedbackPosition() {
  V3d dpos = {state_(static_cast<int>(KfErrorState::kLat)),
              state_(static_cast<int>(KfErrorState::kLon)),
              state_(static_cast<int>(KfErrorState::kAlt))};
  pSINS_->FeedbackVelocity(dpos);
  state_(static_cast<int>(KfErrorState::kLat)) = 0;
  state_(static_cast<int>(KfErrorState::kLon)) = 0;
  state_(static_cast<int>(KfErrorState::kAlt)) = 0;
}
void KF15::FeedbackGyroBias() {
  V3d gyro_bias = {state_(static_cast<int>(KfErrorState::kGbx)),
                   state_(static_cast<int>(KfErrorState::kGby)),
                   state_(static_cast<int>(KfErrorState::kGbz))};
  pSINS_->FeedbackGyroBias(gyro_bias);
  state_(static_cast<int>(KfErrorState::kGbx)) = 0;
  state_(static_cast<int>(KfErrorState::kGby)) = 0;
  state_(static_cast<int>(KfErrorState::kGbz)) = 0;
}
void KF15::FeedbackAcceBias() {
  V3d acce_bias = {state_(static_cast<int>(KfErrorState::kAbx)),
                   state_(static_cast<int>(KfErrorState::kAby)),
                   state_(static_cast<int>(KfErrorState::kAbz))};
  pSINS_->FeedbackVelocity(acce_bias);
  state_(static_cast<int>(KfErrorState::kAbx)) = 0;
  state_(static_cast<int>(KfErrorState::kAby)) = 0;
  state_(static_cast<int>(KfErrorState::kAbz)) = 0;
}

}  // namespace sins
