#include "sensors/imu.h"

IMUData::IMUData(): timestamp_(0){
    gyro_.setZero();
    acce_.setZero();
}

IMUData::IMUData(const V3d &gyro, const V3d &acce, double timestamp):
    timestamp_(timestamp),
    gyro_(gyro),
    acce_(acce){
}

const V3d& IMUData::Gyro() const {
    return gyro_;
}

const V3d& IMUData::Acce() const {
    return acce_;
}

const double IMUData::Timestamp() const {
    return timestamp_;
}