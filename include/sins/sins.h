#ifndef SINS_H_
#define SINS_H_

#include <memory>
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
    SINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts);
	virtual ~SINS(){}
    virtual void Update(const IMUData& imu) = 0;
    virtual void UpdateAttitude() = 0;
    virtual void UpdateVelocity() = 0;
	virtual void UpdatePosition() = 0;
    const V3d& GetAttitude() const;
    const V3d& GetVelocity() const;
    const V3d& GetPosition() const;
    const M3d& GetRotationMatrix() const;

protected:
    V3d att_; //pitch roll yaw
    V3d vn_, last_vn_;
    V3d pos_;
    Eigen::Quaterniond q_,q_prev_;
    double update_timestamp_, last_update_timestamp_, dt_;
    double current_imu_timestamp_, prev_imu_timestamp_;
    double ts_;
};
} // namespace sins
#endif
