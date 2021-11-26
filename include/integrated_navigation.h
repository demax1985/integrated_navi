// Copyright 2021 demax
#ifndef INCLUDE_INTEGRATED_NAVIGATION_H_
#define INCLUDE_INTEGRATED_NAVIGATION_H_

#include <memory>

#include "filter/filter_base.h"
#include "sensors/gnss.h"
#include "sins/sins.h"
using sins::SINS;

class IntegratedNavigation {
 private:
  std::shared_ptr<SINS> pSINS_;
  std::unique_ptr<FilterBase> pFilter_;
  // flags
  bool is_static_;
  bool gnss_vel_valid_, gnss_pos_valid_;

  // measuremet
  GnssData gnss_;
  IMUData imu_;
  double baro_alt_;
  double gnss_yaw_;
  double zihr_initial_yaw_, zihr_initial_time_;

 public:
  IntegratedNavigation(/* args */);
  ~IntegratedNavigation();
};

IntegratedNavigation::IntegratedNavigation(/* args */) {}

IntegratedNavigation::~IntegratedNavigation() {}

#endif  // INCLUDE_INTEGRATED_NAVIGATION_H_
