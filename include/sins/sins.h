#ifndef SINS_H_
#define SINS_H_

#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sensors/imu.h"
namespace sins{

using V3d = Eigen::Vector3d;
using M3d = Eigen::Matrix3d;
class SINS
{
public:
    SINS():
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
    SINS(const V3d& att, const V3d& vel, const V3d& pos):
        attitude_(att),
        velocity_(vel),
        last_velocity_(vel),
        position_(pos),
        update_time_(0),
        last_update_time_(0),
        dt_(0)
    {
        Eigen::AngleAxisd yawAngle(att(2),V3d::UnitZ());
        Eigen::AngleAxisd pitchAngle(att(0),V3d::UnitX());
        Eigen::AngleAxisd rollAngle(att(1),V3d::UnitY());
        q_ = rollAngle*pitchAngle*yawAngle;
    }
	virtual ~SINS(){}
    virtual void Update(const IMUdata& imu) = 0;
    virtual void UpdateAttitude(const V3d& gyro) = 0;
    virtual void UpdateVelocity(const V3d& acce) = 0;
	virtual void UpdatePosition() = 0;
protected:
    V3d attitude_; //pitch roll yaw
    V3d velocity_, last_velocity_;
    V3d position_;
    Eigen::Quaterniond q_;
    double update_time_, last_update_time_, dt_;
};
} // namespace sins
#endif
