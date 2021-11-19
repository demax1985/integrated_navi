
#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using V3d = Eigen::Vector3d;
class IMUData
{
public:
	IMUData();
    IMUData(const V3d& gyro, const V3d& acce, double timestamp);
    ~IMUData(){}
    const V3d& Gyro() const;
    const V3d& Acce() const;
    const double Timestamp() const;
private:
    V3d gyro_;
    V3d acce_;
    double timestamp_;
};

#endif
