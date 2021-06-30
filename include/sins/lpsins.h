#ifndef LPSINS_H_
#define LPSINS_H_

#include "sins.h"
#include "sensors/imu.h"
namespace sins{
class LPSINS : public SINS
{
public:
	LPSINS();
    ~LPSINS(){}
    virtual void Update(const IMUdata& imu);
    virtual void UpdateAttitude(const V3d& gyro);
    virtual void UpdateVelocity(const V3d& acce);
    virtual void UpdatePosition();
private:
    V3d attitude_;
    V3d velocity_, last_velocity_;
    V3d position_;
    Eigen::Quaterniond q_;
    double update_time_, last_update_time_, dt_;
    V3d gn;
};
} //namespace sins
#endif
