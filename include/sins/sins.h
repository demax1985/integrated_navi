#ifndef SINS_H_
#define SINS_H_

#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "sensors/imu.h"
#include "common/global.h"
#include "earth.h"
namespace sins{

using V3d = Eigen::Vector3d;
using M3d = Eigen::Matrix3d;

class SINS
{
public:
    SINS();
    SINS(const V3d& att, const V3d& vel, const V3d& pos, std::shared_ptr<IMUdata> pimu);
	virtual ~SINS(){}
    virtual void Update(const IMUdata& imu) = 0;
    virtual void UpdateAttitude(const V3d& gyro) = 0;
    virtual void UpdateVelocity(const V3d& acce) = 0;
	virtual void UpdatePosition() = 0;
    const V3d& GetAttitude() const;
    const V3d& GetVelocity() const;
    const V3d& GetPosition() const;
    const M3d& GetRotationMatrix() const;
    std::shared_ptr<IMUdata> GetIMUdata() const;

protected:
    V3d attitude_; //pitch roll yaw
    V3d velocity_, last_velocity_;
    V3d position_;
    Eigen::Quaterniond q_;
    double update_time_, last_update_time_, dt_;
    std::shared_ptr<IMUdata> pimu_;

    int nn_; // number of sub samples
    Earth eth;

};
} // namespace sins
#endif
