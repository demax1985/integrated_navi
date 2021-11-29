// Copyright 2021 demax
#ifndef INCLUDE_INTEGRATED_NAVIGATION_H_
#define INCLUDE_INTEGRATED_NAVIGATION_H_

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <iostream>
#include <memory>

#include "filter/filter_base.h"
#include "sensors/gnss.h"
#include "sensors/imu.h"
#include "sins/sins.h"
using sins::SINS;
enum FusionAlgorithm { kFilter, kFactorGraphOptimization };

class IntegratedNavigation {
 private:
  std::shared_ptr<SINS> pSINS_;
  std::unique_ptr<FilterBase> pFilter_;
  // flags
  bool is_static_;
  bool gnss_vel_valid_, gnss_pos_valid_;

  // counts
  int initial_alignment_count_;

  V3d mean_acce_in_b_fram_;
  V3d mean_gyro_static_;

  double kf_predict_dt_, kf_predict_time_prev_;

  // measuremet
  GnssData gnss_;
  IMUData imu_;
  double baro_alt_;
  double gnss_yaw_;
  double zihr_initial_yaw_, zihr_initial_time_;

  // subscribes
  ros::Subscriber imu_sub_;
  ros::Subscriber gnss_sub_;

  ros::NodeHandle nh_;

  std::ofstream outfile_;

  // static consts
  static const int kInitialAlignmentCount = 200;
  static constexpr double kKfPredictDt = 0.1;

 public:
  IntegratedNavigation(/* args */);
  IntegratedNavigation(std::shared_ptr<SINS> sins,
                       std::unique_ptr<FilterBase> filter);
  ~IntegratedNavigation() { outfile_.close(); }

  void ImuCallback(const sensor_msgs::ImuConstPtr& imu);
  // TODO(demax): define gnss msg later
  void GnssCallback(const sensor_msgs::NavSatFixConstPtr& gnss_pos);
};

#endif  // INCLUDE_INTEGRATED_NAVIGATION_H_
