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

void SINS::EarthUpdate(const V3d& pos, const V3d& vn){
    eth.pos = pos;
    eth.vn = vn;
    eth.sl = sin(pos(0)); eth.cl = cos(pos(0)); eth.tl = eth.sl/eth.cl;
    eth.sl2 = eth.sl*eth.sl; eth.sl4 = eth.sl2*eth.sl2;
    double sq = 1-eth.e2*eth.sl2; 
    double RN = eth.Re/sqrt(sq);
    eth.RNh = RN + pos(2); eth.clRNh = eth.cl*eth.RNh;
    eth.RMh = RN*(1-eth.e2)/sq + pos(2);

    eth.wnie(1) = eth.wie*eth.cl; eth.wnie(2) = eth.wie*eth.sl;

    eth.wnen(0) = -vn(1)/eth.RMh; eth.wnen(1) = vn(0)/eth.RNh; eth.wnen(2) = eth.wnen(1)*eth.tl;

    eth.wnin = eth.wnie = eth.wnen;
    eth.wnien = eth.wnie + eth.wnin;
    eth.g = eth.g0*(1+5.27094e-3*eth.sl2+2.32718e-5*eth.sl4)-3.086e-6*pos(2);
    eth.gn(2) = -eth.g;

    eth.gcc = eth.gn - eth.wnien.cross(eth.vn);
}

} //namespace sins
