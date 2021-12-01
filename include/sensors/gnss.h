// Copyright 2021 demax
#ifndef INCLUDE_SENSORS_GNSS_H_
#define INCLUDE_SENSORS_GNSS_H_
#include <Eigen/Core>
using V3d = Eigen::Vector3d;
using M3d = Eigen::Matrix3d;
struct GnssData {
  struct GnssDop {
    double gdop;  // geometric dop
    double pdop;  // position dop
    double tdop;  // time dop
    double vdop;  // vertical dop
    double hdop;  // horizontal dop
    double ndop;  // north dop
    double edop;  // east dop
  };

  double timestamp_;
  double tow_ms_;
  int fixtype_;
  int sv_num_;
  double cnr_mean_, cnr_sigma_;
  double dr_rms_, pr_rms_;
  double sv_block_angle_;
  GnssDop dop_;
  V3d gnss_pos_, gnss_vn_;
  V3d gnss_pos_sigma_, gnss_vn_sigma_;
};

#endif  // INCLUDE_SENSORS_GNSS_H_
