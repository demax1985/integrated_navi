#include "sins/sins.h"

namespace sins {

SINS::SINS():
    update_time_(0),
    last_update_time_(0),
    dt_(0)
{
    attitude_.setZero();
    velocity_.setZero();
    last_velocity_.setZero();
    position_.setZero();
    q_.setIdentity();
}

SINS::SINS(const V3d& att, const V3d& vel, const V3d& pos, std::shared_ptr<IMUdata> pimu):
    attitude_(att),
    velocity_(vel),
    last_velocity_(vel),
    position_(pos),
    update_time_(0),
    last_update_time_(0),
    dt_(0)
{
    pimu_ = pimu;
    Eigen::AngleAxisd yawAngle(att(2),V3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(att(0),V3d::UnitX());
    Eigen::AngleAxisd rollAngle(att(1),V3d::UnitY());
    q_ = rollAngle*pitchAngle*yawAngle;
}

const V3d& SINS::GetAttitude() const {
    return attitude_;
}

const V3d& SINS::GetVelocity() const {
    return velocity_;
}

const V3d& SINS::GetPosition() const {
    return position_;
}

const M3d& SINS::GetRotationMatrix() const {
    return std::move(q_.toRotationMatrix());
}

std::shared_ptr<IMUdata> SINS::GetIMUdata() const{
    return pimu_;
}

} //namespace sins
