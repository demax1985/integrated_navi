// Copyright 2021 demax
#include "integrated_navigation.h"

IntegratedNavigation::IntegratedNavigation()
    : is_static_(false),
      gnss_vel_valid_(false),
      gnss_pos_valid_(false),
      baro_alt_(0.0),
      gnss_yaw_(0.0),
      zihr_initial_yaw_(0.0),
      zihr_initial_time_(0.0) {
  imu_sub_ =
      nh_.subscribe("/IMU_data", &IntegratedNavigation::ImuCallback, this);
  gnss_sub_ =
      nh_.subscribe("/gnss_data", &IntegratedNavigation::GnssCallback, this);
}
