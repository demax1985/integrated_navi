// Copyright 2021 demax
#include <ros/ros.h>

#include <memory>
#include <unsupported/Eigen/MatrixFunctions>

#include "../../include/sins/hpsins.h"
#include "common/global.h"
#include "filter/kf15.h"
#include "integrated_navigation.h"
#include "sensors/gnss.h"
#include "sensors/imu.h"

using sins::HPSINS;
using sins::KF15;

const double kDeg = kPi / 180.0;
const double kMg = 9.7803267714 * 0.001;
const double kUg = 9.7803267714 * 1.0e-6;
const double kDph = kDeg / 3600.0;
const double kRe = 6378137.0;
const double kDpsh = kDeg / sqrt(3600.0);
const double kUgpsHz = kUg;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_static_node");

  ros::NodeHandle nh;
  ros::Publisher imu_pub, gnss_pub;
  imu_pub = nh.advertise<sensor_msgs::Imu>("/IMU_data", 1000);
  gnss_pub = nh.advertise<sensor_msgs::NavSatFix>("/gnss_data", 1000);

  // test kf15
  V3d att = {0, 0, 0};        // true att
  V3d vn = {0, 0, 0};         // true vn
  V3d pos = {0.6, 2.4, 100};  // true pos

  V3d att_error = {0.1, 0.1, 1.0};
  V3d vn_error = {0.1, 0.1, 0.1};
  V3d pos_error = {0, 0, 0};

  V3d eb = {75.0 * kDph, 75.0 * kDph, 75.0 * kDph};
  V3d db = {10.0 * kMg, 10.0 * kMg, 10.0 * kMg};
  V3d web = {0.01 * kDpsh, 0.01 * kDpsh, 0.01 * kDpsh};
  V3d wdb = {100 * kUgpsHz, 100 * kUgpsHz, 100 * kUgpsHz};

  V3d ini_att = att + att_error;
  V3d ini_vn = vn + vn_error;
  V3d ini_pos = pos + pos_error;
  double ts = 0.01;
  double n = 1;
  double taug = 3600;
  double taua = 3600;
  std::shared_ptr<HPSINS> psins(
      new HPSINS(ini_att, ini_vn, ini_pos, ts, n, taug, taua));

  // set kf
  Eigen::Matrix<double, 15, 15> pk;
  Eigen::Matrix<double, 15, 15> qt;
  Eigen::Matrix<double, 15, 1> tmppk;
  tmppk << 10 * kDeg, 10 * kDeg, 10 * kDeg, 1, 1, 1, 10 / kRe, 10 / kRe, 10.0,
      75 * kDph, 75 * kDph, 75 * kDph, 10 * kMg, 10 * kMg, 10 * kMg;
  pk = tmppk.asDiagonal();

  Eigen::Matrix<double, 15, 1> tmpqt;
  tmpqt << web(0), web(1), web(2), wdb(0), wdb(1), wdb(2), 0, 0, 0, 0, 0, 0, 0,
      0, 0;
  qt = tmpqt.asDiagonal();
  Eigen::Matrix<double, 15, 1> state;
  state.setZero();

  std::unique_ptr<KF15> pkf15(new KF15(state, pk, qt, psins));
  IntegratedNavigation estimate(psins, pkf15);
}