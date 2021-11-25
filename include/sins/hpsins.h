// Copyright 2021 demax
#ifndef INCLUDE_SINS_HPSINS_H_
#define INCLUDE_SINS_HPSINS_H_

#include <memory>
#include <tuple>
#include <vector>

#include "sins/sins.h"
namespace sins {

using std::cout;
using std::endl;

class HPSINS : public SINS {
 private:
  int num_samples_;  // number of sub samples
  V3d phim_, phim_prev_;
  V3d dvbm_, dvbm_prev_;
  V3d wib_, wib_prev_, wib_middle_;
  V3d fb_, fb_prev_, fb_middle_;
  V3d pos_middle_, vn_middle_;
  Eigen::Quaterniond q_middle_;
  std::unique_ptr<Earth> eth_;
  std::vector<IMUData> imus_;
  Eigen::Matrix<double, 5, 5> cone_scull_coeff_;

  double tauG_, tauA_;

 public:
  HPSINS(/* args */);
  HPSINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts,
         const int num_samples, double tauG, double tauA);
  HPSINS(const HPSINS& other);             // copy constructor
  HPSINS& operator=(const HPSINS& other);  // operator =

  void Update(const IMUData& imu) override;
  void UpdateAttitude() override;
  void UpdateVelocity() override;
  void UpdatePosition() override;

  void SetErrModelMatrix() override;

  const M3d& Maa() const override;
  const M3d& Mav() const override;
  const M3d& Map() const override;
  const M3d& Mva() const override;
  const M3d& Mvv() const override;
  const M3d& Mvp() const override;
  const M3d& Mpv() const override;
  const M3d& Mpp() const override;

  const double TauG() const override;
  const double TauA() const override;

  void FeedbackAttitude(const V3d& phi) override;
  void FeedbackVelocity(const V3d& dvn) override;
  void FeedbackPosition(const V3d& dpos) override;
  void FeedbackGyroBias(const V3d& gyro_bias) override;
  void FeedbackAcceBias(const V3d& acce_bias) override;

  void InitialLevelAlignment(const V3d& mean_acce_in_b_fram) override;

  const V3d& EarthWnin() const;
  const V3d& EarthWnie() const;
  const V3d& EarthWnien() const;
  const V3d& EarthGcc() const;
  double EarthRmh() const;
  double EarthRnh() const;
  double EarthTl() const;
  double EarthSl() const;
  double EarthCl() const;
  double EarthClRnh() const;
  double EarthG0() const;

  void ConeScullCompensation();
  void UpdatePrevSINS();
  const V3d Vn2DeltaPos(const V3d& vn, double dt) const;
  void ComputeWibAndFb();
  std::tuple<V3d, V3d> ExtrapolatePosAndVn(double dt);

  void ShowAtt() const {
    cout << "euler angle is: " << att_ << endl;
    cout << " quaternion is: " << q_.w() << " " << q_.x() << " " << q_.y()
         << " " << q_.z() << endl;
  }
  void ShowVel() const { cout << "velocity is: " << vn_ << endl; }
  void ShowPos() const { cout << "position is: " << pos_ << endl; }
};

}  // namespace sins

#endif  // INCLUDE_SINS_HPSINS_H_
