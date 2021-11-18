#ifndef SINS_HPSINS_H_
#define SINS_HPSINS_H_

#include "sins/sins.h"
#include <memory>
namespace sins{

using std::cout;
using std::endl;

class HPSINS : public SINS
{
private:
    int num_samples_; // number of sub samples
    V3d phim_,prev_phim_;
    V3d dvbm_,prev_dvbm_;
    std::unique_ptr<Earth> eth_;
    std::vector<IMUData> imus_;
    Eigen::Matrix<double,5,5> cone_scull_coeff_;
public:
    HPSINS(/* args */);
    HPSINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts, const int num_samples);

    virtual void Update(const IMUData& imu) override;
    virtual void UpdateAttitude() override;
    virtual void UpdateVelocity() override;
	virtual void UpdatePosition() override;

    void ConeScullCompensation();

    void ShowAtt() const {cout<<"euler angle is: "<<att_<<endl;
                          cout<<" quaternion is: "<<q_.w()<<" "<<q_.x()<<" "<<q_.y()<<" "<<q_.z()<<endl;}
    void ShowVel() const {cout<<"velocity is: "<<vn_<<endl;}
    void ShowPos() const {cout<<"position is: "<<pos_<<endl;}
};

} //namespace sins











#endif