#include "sins/sins.h"

namespace sins {

SINS::SINS():
    update_timestamp_(0),
    pre_update_timestamp_(0),
    dt_(0),
    ts_(0.01),
    current_imu_timestamp_(0.0),
    prev_imu_timestamp_(0.0)
{
    att_.setZero();
    vn_.setZero();
    vn_prev_.setZero();
    an_.setZero();
    pos_.setZero();
    q_.setIdentity();
    q_prev_.setIdentity();
}

SINS::SINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts):
    att_(att),
    vn_(vn),
    vn_prev_(vn),
    pos_(pos),
    update_timestamp_(0),
    pre_update_timestamp_(0),
    dt_(0),
    ts_(ts),
    current_imu_timestamp_(0.0),
    prev_imu_timestamp_(0.0)
{
    Eigen::AngleAxisd yawAngle(att(2),V3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(att(0),V3d::UnitX());
    Eigen::AngleAxisd rollAngle(att(1),V3d::UnitY());
    q_ = rollAngle*pitchAngle*yawAngle;
    q_prev_ = q_;
    an_.setZero();
}

const V3d& SINS::GetAttitude() const {
    return att_;
}

const V3d& SINS::GetVelocity() const {
    return vn_;
}

const V3d& SINS::GetPosition() const {
    return pos_;
}

const M3d SINS::GetRotationMatrix() const {
    return q_.toRotationMatrix();
}


} //namespace sins
