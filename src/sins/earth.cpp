// Copyright 2021 demax
#include "sins/earth.h"

Earth::Earth(/* args */) {
  pos_ = {0.0, 0.0, 0.0};
  vn_ = {0.0, 0.0, 0.0};

  Re_ = 6378137.0;
  double f = 1.0 / 298.257;
  double e = sqrt(2 * f - f * f);
  e2_ = e * e;
  sl_ = sin(pos_(0));
  cl_ = cos(pos_(0));
  tl_ = sl_ / cl_;
  sl2_ = sl_ * sl_;
  double sq = 1.0 - e2_ * sl2_;
  rnh_ = Re_ / sqrt(sq) + pos_(2);
  cl_rnh_ = cl_ * rnh_;
  rmh_ = Re_ / sqrt(sq) * (1.0 - e2_) / sq + pos_(2);
  wie_ = 7.2921151467e-5;
  g0_ = 9.7803267714;
  g_ = g0_ * (1.0 + 5.27094e-3 * sl2_ + 2.32718e-5 * sl2_ * sl2_) -
       3.086e-6 * pos_(2);

  wnie_ = {0.0, wie_ * cl_, wie_ * sl_};
  wnen_ = {-vn_(1) / rmh_, vn_(0) / rnh_, vn_(0) * tl_ / rnh_};
  wnin_ = wnie_ + wnen_;
  wnien_ = wnie_ + wnin_;
  gn_ = {0.0, 0.0, -g_};
  gcc_ = gn_ - wnien_.cross(vn_);
}

Earth::Earth(const V3d& pos, const V3d& vn) {
  pos_ = pos;
  vn_ = vn;
  std::cout << "vn in earth construct is: " << std::endl;
  std::cout << vn_ << std::endl;
  Re_ = 6378137.0;
  double f = 1.0 / 298.257;
  double e = sqrt(2 * f - f * f);
  e2_ = e * e;
  sl_ = sin(pos_(0));
  cl_ = cos(pos_(0));
  tl_ = sl_ / cl_;
  sl2_ = sl_ * sl_;
  double sq = 1.0 - e2_ * sl2_;
  rnh_ = Re_ / sqrt(sq) + pos_(2);
  cl_rnh_ = cl_ * rnh_;
  rmh_ = Re_ / sqrt(sq) * (1.0 - e2_) / sq + pos_(2);
  wie_ = 7.2921151467e-5;
  g0_ = 9.7803267714;
  g_ = g0_ * (1.0 + 5.27094e-3 * sl2_ + 2.32718e-5 * sl2_ * sl2_) -
       3.086e-6 * pos_(2);

  wnie_ = {0.0, wie_ * cl_, wie_ * sl_};
  wnen_ = {-vn_(1) / rmh_, vn_(0) / rnh_, vn_(0) * tl_ / rnh_};
  wnin_ = wnie_ + wnen_;
  wnien_ = wnie_ + wnin_;
  gn_ = {0.0, 0.0, -g_};
  gcc_ = gn_ - wnien_.cross(vn_);
  std::cout << "wnie_ in earth construct is: " << std::endl;
  std::cout << wnie_ << std::endl;
  std::cout << "wnen_ in earth construct is: " << std::endl;
  std::cout << wnen_ << std::endl;
  std::cout << "wnin_ in earth construct is: " << std::endl;
  std::cout << wnin_ << std::endl;
}

void Earth::EarthUpdate(const V3d& pos, const V3d& vn) {
  pos_ = pos;
  vn_ = vn;

  Re_ = 6378137.0;
  double f = 1.0 / 298.257;
  double e = sqrt(2 * f - f * f);
  e2_ = e * e;
  sl_ = sin(pos_(0));
  cl_ = cos(pos_(0));
  tl_ = sl_ / cl_;
  sl2_ = sl_ * sl_;
  double sq = 1.0 - e2_ * sl2_;
  rnh_ = Re_ / sqrt(sq) + pos_(2);
  cl_rnh_ = cl_ * rnh_;
  rmh_ = Re_ / sqrt(sq) * (1.0 - e2_) / sq + pos_(2);
  wie_ = 7.2921151467e-5;
  g0_ = 9.7803267714;
  g_ = g0_ * (1.0 + 5.27094e-3 * sl2_ + 2.32718e-5 * sl2_ * sl2_) -
       3.086e-6 * pos_(2);

  wnie_ = {0.0, wie_ * cl_, wie_ * sl_};
  wnen_ = {-vn_(1) / rmh_, vn_(0) / rnh_, vn_(0) * tl_ / rnh_};
  wnin_ = wnie_ + wnen_;
  wnien_ = wnie_ + wnin_;
  gn_ = {0.0, 0.0, -g_};
  gcc_ = gn_ - wnien_.cross(vn_);
}

const V3d& Earth::Wnin() const { return wnin_; }

const V3d& Earth::Wnie() const { return wnie_; }

const V3d& Earth::Wnien() const { return wnien_; }

const V3d& Earth::Gcc() const { return gcc_; }

const double Earth::Rmh() const { return rmh_; }

const double Earth::Rnh() const { return rnh_; }

const double Earth::ClRnh() const { return cl_rnh_; }

const double Earth::Tl() const { return tl_; }

const double Earth::Sl() const { return sl_; }

const double Earth::Cl() const { return cl_; }

const double Earth::G0() const { return g0_; }
