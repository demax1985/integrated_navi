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
	SINS(){}
	virtual ~SINS(){}
    virtual void Update(const IMUdata& imu) = 0;
    virtual void UpdateAttitude(const V3d& gyro) = 0;
    virtual void UpdateVelocity(const V3d& acce) = 0;
	virtual void UpdatePosition() = 0;
	
};
} // namespace sins
#endif
