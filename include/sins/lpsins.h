#ifndef LPSINS_H_
#define LPSINS_H_

#include "sins.h"
#include "sensors/imu.h"
namespace sins{
using std::cout;
using std::endl;
class LPSINS : public SINS
{
public:
	LPSINS();
    LPSINS(const V3d& att, const V3d& vel, const V3d& pos);
    ~LPSINS(){}
    virtual void Update(const IMUdata& imu);
    virtual void UpdateAttitude(const V3d& gyro);
    virtual void UpdateVelocity(const V3d& acce);
    virtual void UpdatePosition();

    void ShowAtt() const {cout<<"euler angle is: "<<attitude_<<endl;
                          cout<<" quaternion is: "<<q_.w()<<" "<<q_.x()<<" "<<q_.y()<<" "<<q_.z()<<endl;}
    void ShowVel() const {cout<<"velocity is: "<<velocity_<<endl;}
    void ShowPos() const {cout<<"position is: "<<position_<<endl;}
private:
    V3d gn;
};
} //namespace sins
#endif
