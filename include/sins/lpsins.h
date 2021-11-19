#ifndef SINS_LPSINS_H_
#define SINS_LPSINS_H_

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
    virtual void Update();
    virtual void UpdateAttitude();
    virtual void UpdateVelocity();
    virtual void UpdatePosition();

    void ShowAtt() const {cout<<"euler angle is: "<<att_<<endl;
                          cout<<" quaternion is: "<<q_.w()<<" "<<q_.x()<<" "<<q_.y()<<" "<<q_.z()<<endl;}
    void ShowVel() const {cout<<"velocity is: "<<vn_<<endl;}
    void ShowPos() const {cout<<"position is: "<<pos_<<endl;}
private:
    V3d gn;
};
} //namespace sins
#endif
