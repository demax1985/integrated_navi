// Copyright 2021 demax
#ifndef INCLUDE_SINS_LPSINS_H_
#define INCLUDE_SINS_LPSINS_H_

#include "sensors/imu.h"
#include "sins/sins.h"
namespace sins {
using std::cout;
using std::endl;
class LPSINS : public SINS {
 public:
  LPSINS();
  LPSINS(const V3d& att, const V3d& vn, const V3d& pos, double ts, double taug,
         double taua);
  LPSINS(const LPSINS& other);
  ~LPSINS() {}
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

  double EarthRmh() const;
  double EarthRnh() const;
  double EarthClRnh() const;
  const V3d& EarthGcc() const;

 private:
  V3d gn;
  std::unique_ptr<Earth> eth_;

  const V3d Vn2DeltaPos(const V3d& vn, double dt) const;
};
}  // namespace sins
#endif  // INCLUDE_SINS_LPSINS_H_
