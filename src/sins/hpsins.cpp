// Copyright 2021 demax
#include "sins/hpsins.h"

namespace sins {

HPSINS::HPSINS() : SINS(), tauG_(3600.0), tauA_(3600.0) {
  imus_.clear();
  eth_ = std::unique_ptr<Earth>(new Earth());
  phim_.setZero();
  dvbm_.setZero();
  phim_prev_.setZero();
  dvbm_prev_.setZero();
  wib_.setZero();
  wib_prev_.setZero();
  wib_middle_.setZero();
  fb_.setZero();
  fb_prev_.setZero();
  fb_middle_.setZero();
  pos_middle_.setZero();
  vn_middle_.setZero();
  q_middle_ = q_;
  cone_scull_coeff_ << 2 / 3, 0, 0, 0, 0, 9 / 20, 27 / 20, 0, 0, 0, 54 / 105,
      92 / 105, 214 / 105, 0, 0, 250 / 504, 525 / 504, 650 / 504, 1375 / 504, 0,
      2315 / 4620, 4558 / 4620, 7296 / 4620, 7834 / 4620, 15797 / 4620;
}

HPSINS::HPSINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts,
               const int num_samples, double tauG, double tauA)
    : SINS(att, vn, pos, ts),
      num_samples_(num_samples),
      tauG_(tauG),
      tauA_(tauA) {
  eth_ = std::unique_ptr<Earth>(new Earth(pos, vn));
  imus_.clear();
  phim_.setZero();
  dvbm_.setZero();
  phim_prev_.setZero();
  dvbm_prev_.setZero();
  wib_.setZero();
  wib_prev_.setZero();
  wib_middle_.setZero();
  fb_.setZero();
  fb_prev_.setZero();
  fb_middle_.setZero();
  pos_middle_ = pos_;
  vn_middle_ = vn_;
  q_middle_ = q_;
  cone_scull_coeff_ << 2 / 3, 0, 0, 0, 0, 9 / 20, 27 / 20, 0, 0, 0, 54 / 105,
      92 / 105, 214 / 105, 0, 0, 250 / 504, 525 / 504, 650 / 504, 1375 / 504, 0,
      2315 / 4620, 4558 / 4620, 7296 / 4620, 7834 / 4620, 15797 / 4620;
}

HPSINS::HPSINS(const HPSINS& other) : SINS(other) {
  std::cout << "HPSINS copy constructor is called" << std::endl;
  num_samples_ = other.num_samples_;
  phim_ = other.phim_;
  phim_prev_ = other.phim_prev_;
  dvbm_ = other.dvbm_;
  dvbm_prev_ = other.dvbm_prev_;
  wib_ = other.wib_;
  wib_prev_ = other.wib_prev_;
  wib_middle_ = other.wib_middle_;
  fb_ = other.fb_;
  fb_prev_ = other.fb_prev_;
  fb_middle_ = other.fb_middle_;
  pos_middle_ = other.pos_middle_;
  vn_middle_ = other.vn_middle_;
  q_middle_ = other.q_middle_;

  imus_ = other.imus_;
  cone_scull_coeff_ = other.cone_scull_coeff_;

  tauG_ = other.tauG_;
  tauA_ = other.tauA_;

  eth_ = std::unique_ptr<Earth>(new Earth(*other.eth_));
}

HPSINS& HPSINS::operator=(const HPSINS& other) {
  std::cout << "HPSINS operator = is called" << std::endl;
  if (this == &other) {
    return *this;
  }
  SINS::operator=(other);
  num_samples_ = other.num_samples_;
  phim_ = other.phim_;
  phim_prev_ = other.phim_prev_;
  dvbm_ = other.dvbm_;
  dvbm_prev_ = other.dvbm_prev_;
  wib_ = other.wib_;
  wib_prev_ = other.wib_prev_;
  wib_middle_ = other.wib_middle_;
  fb_ = other.fb_;
  fb_prev_ = other.fb_prev_;
  fb_middle_ = other.fb_middle_;
  pos_middle_ = other.pos_middle_;
  vn_middle_ = other.vn_middle_;
  q_middle_ = other.q_middle_;

  imus_ = other.imus_;
  cone_scull_coeff_ = other.cone_scull_coeff_;

  tauG_ = other.tauG_;
  tauA_ = other.tauA_;

  eth_.reset(new Earth(*other.eth_));
  return *this;
}

void HPSINS::Update(const IMUData& imu) {
  current_imu_timestamp_ = imu.Timestamp();
  if (current_imu_timestamp_ > prev_imu_timestamp_) {
    if (imus_.size() < num_samples_) {
      imus_.emplace_back(imu);
    } else {
      imus_.clear();
      imus_.emplace_back(imu);
    }
  }
  if (imus_.size() == num_samples_) {
    update_timestamp_ = imus_.back().Timestamp();
    dt_ = update_timestamp_ - pre_update_timestamp_;
    ConeScullCompensation();
    ComputeWibAndFb();
    V3d extrapolated_pos, extrapolated_vn;
    std::tie(extrapolated_pos, extrapolated_vn) =
        ExtrapolatePosAndVn(dt_ / 2.0);
    eth_->EarthUpdate(extrapolated_pos, extrapolated_vn);
    UpdateAttitude();
    UpdateVelocity();
    UpdatePosition();
    pre_update_timestamp_ = update_timestamp_;
  }
}

void HPSINS::UpdateAttitude() {
  // Eigen::AngleAxisd phi_n_in = V3d2AngleAxisd(-eth_->Wnin()*dt_);
  // Eigen::AngleAxisd phi_b_ib = V3d2AngleAxisd((phim_ + prev_phim_)/2);
  // Eigen::Quaterniond q_n_in(phi_n_in);
  // Eigen::Quaterniond q_b_ib(phi_b_ib);

  Eigen::Quaterniond q_n_in = RotationVector2Quaternion(EarthWnin() * dt_);
  Eigen::Quaterniond q_b_ib = RotationVector2Quaternion(wib_middle_ * dt_);
  q_ = q_n_in * q_ * q_b_ib;

  Eigen::Quaterniond q_n_in_middle =
      RotationVector2Quaternion(EarthWnin() * dt_ / 2.0);
  Eigen::Quaterniond q_b_ib_middle =
      RotationVector2Quaternion(wib_middle_ * dt_ / 2.0);
  q_middle_ = q_n_in_middle * q_ * q_b_ib_middle;
  // TODO(demax) : check eulerangle rotation sequence
  // att_ = q_.matrix().eulerAngles(2, 1, 0);
  att_ = Quaternion2Euler(q_);
}

void HPSINS::UpdateVelocity() {
  an_ = q_middle_ * fb_middle_ + EarthGcc();
  vn_ += an_ * dt_;
  vn_middle_ = (vn_prev_ + vn_) / 2.0;
}

void HPSINS::UpdatePosition() { pos_ += Vn2DeltaPos(vn_middle_, dt_); }

// TODO(demax) : ts_ is not correct,should be the time interval between two
// consecutive imu data
void HPSINS::ConeScullCompensation() {
  if (imus_.size() < num_samples_) return;

  phim_.setZero();
  dvbm_.setZero();
  V3d cm(0, 0, 0), sm(0, 0, 0), wm(0, 0, 0),
      vm(0, 0, 0);  // attention initialized to zero
  for (int i = 0; i < num_samples_ - 1; i++) {
    cm += cone_scull_coeff_(num_samples_ - 2, i) * imus_.at(i).Gyro() * ts_;
    sm += cone_scull_coeff_(num_samples_ - 2, i) * imus_.at(i).Acce() * ts_;
  }

  std::for_each(imus_.begin(), imus_.end(),
                [&wm, this](const IMUData& imu) { wm += imu.Gyro() * ts_; });
  std::for_each(imus_.begin(), imus_.end(),
                [&vm, this](const IMUData& imu) { vm += imu.Acce() * ts_; });

  phim_ = wm + cm.cross(imus_.at(num_samples_ - 1).Gyro() *
                        ts_);  // coning error compensation

  // sculling error compensation
  dvbm_ += cm.cross(imus_.at(num_samples_ - 1).Acce() * ts_) +
           sm.cross(imus_.at(num_samples_ - 1).Gyro() * ts_);
  dvbm_ += 0.5 * wm.cross(vm);  // rot error compensation
}

void HPSINS::UpdatePrevSINS() {
  q_prev_ = q_;
  vn_prev_ = vn_;
  phim_prev_ = phim_;
  dvbm_prev_ = dvbm_;
  wib_prev_ = wib_;
  fb_prev_ = fb_;
}

const V3d HPSINS::Vn2DeltaPos(const V3d& vn, double dt) const {
  return V3d(vn(1) * dt / EarthRmh(), vn(0) * dt / EarthClRnh(), vn(2) * dt);
}

void HPSINS::ComputeWibAndFb() {
  wib_ = phim_ / dt_;
  fb_ = dvbm_ / dt_;
  wib_middle_ = (wib_prev_ + wib_) / 2.0;
  fb_middle_ = (fb_prev_ + fb_) / 2.0;
}

std::tuple<V3d, V3d> HPSINS::ExtrapolatePosAndVn(double dt) {
  V3d extrapolated_vn = vn_ + an_ * dt;
  V3d extrapolated_pos = pos_ + Vn2DeltaPos(vn_, dt);
  return std::make_tuple(extrapolated_pos, extrapolated_vn);
}

void HPSINS::SetErrModelMatrix() {
  double tl = EarthTl();
  double secl = 1.0 / EarthCl();
  double f_Rmh = 1.0 / EarthRmh();
  double f_Rnh = 1.0 / EarthRnh();
  double f_clRnh = 1.0 / EarthClRnh();
  double f_Rmh2 = f_Rmh * f_Rmh;
  double f_Rnh2 = f_Rnh * f_Rnh;
  M3d Mp1, Mp2;
  Mp1.setZero();
  Mp2.setZero();
  Mp1(1, 0) = -EarthWnie()(2);
  Mp1(2, 0) = EarthWnie()(1);
  Mp2(0, 2) = vn_(1) * f_Rmh2;
  Mp2(1, 2) = -vn_(0) * f_Rnh2;
  Mp2(2, 0) = vn_(0) * f_clRnh * secl;
  Mp2(2, 2) = -vn_(0) * f_Rnh2 * tl;

  V3d fn = q_ * fb_;
  double scl = EarthSl() * EarthCl();

  Maa_ = -V3d2Skew(EarthWnin());
  Mav_(0, 1) = -f_Rmh;
  Mav_(1, 0) = f_Rnh;
  Mav_(2, 0) = f_Rnh * tl;
  Map_ = Mp1 + Mp2;
  Mva_ = V3d2Skew(fn);
  Mvv_ = V3d2Skew(vn_) * Mav_ - V3d2Skew(EarthWnien());
  Mvp_ = V3d2Skew(vn_) * (Mp1 + Map_);
  Mvp_(2, 0) -= EarthG0() *
                (5.27094e-3 * 2.0 + 2.32718e-5 * 4.0 * EarthSl() * EarthSl()) *
                scl;
  Mvp_(2, 2) += 3.086e-6;
  Mpv_(0, 1) = f_Rmh;
  Mpv_(1, 0) = f_clRnh;
  Mpv_(2, 2) = 1.0;
  Mpp_(0, 2) = -vn_(1) * f_Rmh2;
  Mpp_(1, 0) = vn_(0) * f_clRnh * tl;
  Mpp_(1, 2) = -vn_(0) * f_Rnh2 * secl;
}

void HPSINS::InitialLevelAlignment(const V3d& mean_acce_in_b_fram) {
  double pitch = asin(mean_acce_in_b_fram(1) / EarthG0());
  double roll = atan2(-mean_acce_in_b_fram(0), mean_acce_in_b_fram(2));
  att_ = {pitch, roll, 0};
  q_ = Euler2Quaternion(att_);
  q_prev_ = q_;
  SetInitStatus(true);
}

void HPSINS::FeedbackAttitude(const V3d& phi) {
  q_ = RotationVector2Quaternion(phi) * q_;
}
void HPSINS::FeedbackVelocity(const V3d& dvn) { vn_ -= dvn; }
void HPSINS::FeedbackPosition(const V3d& dpos) { pos_ -= dpos; }
void HPSINS::FeedbackGyroBias(const V3d& gyro_bias) { gyro_bias_ += gyro_bias; }
void HPSINS::FeedbackAcceBias(const V3d& acce_bias) { acce_bias_ += acce_bias; }

const V3d& HPSINS::EarthWnin() const { return eth_->Wnin(); }
const V3d& HPSINS::EarthWnie() const { return eth_->Wnie(); }
const V3d& HPSINS::EarthWnien() const { return eth_->Wnien(); }
const V3d& HPSINS::EarthGcc() const { return eth_->Gcc(); }
double HPSINS::EarthRmh() const { return eth_->Rmh(); }
double HPSINS::EarthRnh() const { return eth_->Rnh(); }
double HPSINS::EarthTl() const { return eth_->Tl(); }
double HPSINS::EarthSl() const { return eth_->Sl(); }
double HPSINS::EarthCl() const { return eth_->Cl(); }
double HPSINS::EarthClRnh() const { return eth_->Rnh(); }
double HPSINS::EarthG0() const { return eth_->G0(); }

const M3d& HPSINS::Maa() const { return Maa_; }
const M3d& HPSINS::Mav() const { return Mav_; }
const M3d& HPSINS::Map() const { return Map_; }
const M3d& HPSINS::Mva() const { return Mva_; }
const M3d& HPSINS::Mvv() const { return Mvv_; }
const M3d& HPSINS::Mvp() const { return Mvp_; }
const M3d& HPSINS::Mpv() const { return Mpv_; }
const M3d& HPSINS::Mpp() const { return Mpp_; }

const double HPSINS::TauG() const { return tauG_; }

const double HPSINS::TauA() const { return tauA_; }
}  // namespace sins
