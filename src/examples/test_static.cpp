// Copyright 2021 demax
#include <ros/ros.h>

#include <ctime>
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

ros::Publisher imu_pub, gnss_pub;

double RandomData() { return rand() % 10000 / (double)10000; }

void PublishImuData(const V3d& gyro, const V3d& acce) {
  sensor_msgs::Imu imu;
  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = "imu";
  imu.angular_velocity.x = gyro(0);
  imu.angular_velocity.y = gyro(1);
  imu.angular_velocity.z = gyro(2);

  imu.linear_acceleration.x = acce(0);
  imu.linear_acceleration.y = acce(1);
  imu.linear_acceleration.z = acce(2);
  imu_pub.publish(imu);
}

void PublishGnssData(const V3d& pos, const V3d& rk) {
  sensor_msgs::NavSatFix gnss;
  gnss.header.frame_id = "gnss";
  gnss.header.stamp = ros::Time::now();

  gnss.latitude = pos(0);
  gnss.longitude = pos(1);
  gnss.altitude = pos(2);

  gnss.position_covariance.at(0) = rk(0);
  gnss.position_covariance.at(4) = rk(1);
  gnss.position_covariance.at(8) = rk(2);
  gnss_pub.publish(gnss);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_static_node");
  ros::NodeHandle nh;
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
  double n = 1;  // subsample number
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

  psins->SetErrModelMatrix();
  Eigen::Matrix3d Rk;
  V3d tmprk(0.1, 0.1, 1);
  tmprk = psins->Mpv() * tmprk;
  tmprk = {tmprk(0) * tmprk(0), tmprk(1) * tmprk(1), tmprk(2) * tmprk(2)};
  //   std::cout.precision(10);

  Rk = tmprk.asDiagonal();
  std::cout << "rk is: " << Rk << std::endl;

  std::unique_ptr<KF15> pkf15(new KF15(state, pk, qt, psins));
  IntegratedNavigation estimate(psins, std::move(pkf15));

  ros::Rate rate(100);
  int cnt = 0;

  while (ros::ok()) {
    // simulate imu and gnss data
    V3d gyro =
        eb + V3d(web(0) * RandomData() * 10.0, web(1) * RandomData() * 10.0,
                 web(2) * RandomData() * 10.0);
    V3d acce = V3d(0, 0, 9.7803267714) + db +
               V3d(wdb(0) * RandomData() * 10.0, wdb(1) * RandomData() * 10.0,
                   wdb(2) * RandomData() * 10.0);
    V3d gnss = V3d{sqrt(Rk(0, 0)) * RandomData(), sqrt(Rk(1, 1)) * RandomData(),
                   sqrt(Rk(2, 2)) * RandomData()} +
               pos;
    // std::cout << "gyro is: " << gyro << std::endl;
    // std::cout << "acce is: " << acce << std::endl;
    // std::cout.precision(10);
    // std::cout << "gnss is: " << gnss << std::endl;
    PublishImuData(gyro, acce);
    if (100 == cnt++) {
      PublishGnssData(gnss, tmprk);
      //   std::cout << "gnss publish" << std::endl;
      cnt = 0;
    }
    ros::spinOnce();
    rate.sleep();
  }
}