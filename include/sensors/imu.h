
#ifndef IMU_H_
#define IMU_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using V3d = Eigen::Vector3d;
class IMUdata
{
public:
	IMUdata();
    IMUdata(const V3d& gyro, const V3d& acce, double time);
    ~IMUdata(){}
    const V3d& Gyro() const;
    const V3d& Acce() const;
private:
    V3d gyro_;
    V3d acce_;
    double time_;
};

#endif
