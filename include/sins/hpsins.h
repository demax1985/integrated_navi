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
    V3d phim_,phim_prev_;
    V3d dvbm_,dvbm_prev_;
    V3d wib_,wib_prev_,wib_middle_;
    V3d fb_,fb_prev_,fb_middle_;
    V3d pos_middle_,vn_middle_;
    Eigen::Quaterniond q_middle_;
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
    void UpdatePrevSINS();
    const V3d Vn2DeltaPos(const V3d& vn, double dt) const;
    void ComputeWibAndFb();

    void ShowAtt() const {cout<<"euler angle is: "<<att_<<endl;
                          cout<<" quaternion is: "<<q_.w()<<" "<<q_.x()<<" "<<q_.y()<<" "<<q_.z()<<endl;}
    void ShowVel() const {cout<<"velocity is: "<<vn_<<endl;}
    void ShowPos() const {cout<<"position is: "<<pos_<<endl;}
};

} //namespace sins











#endif