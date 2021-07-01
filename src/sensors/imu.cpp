#include "sensors/imu.h"

IMUdata::IMUdata(): time_(0){
    gyro_.setZero();
    acce_.setZero();
}

IMUdata::IMUdata(const V3d &gyro, const V3d &acce, double time):
    time_(time),
    gyro_(gyro),
    acce_(acce){
}

const V3d& IMUdata::Gyro() const {
    return gyro_;
}

const V3d& IMUdata::Acce() const {
    return acce_;
}
