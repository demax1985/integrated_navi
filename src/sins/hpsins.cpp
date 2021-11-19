#include "sins/hpsins.h"

namespace sins{

HPSINS::HPSINS():SINS(){
    imus_.clear();
    eth_ = std::unique_ptr<Earth>(new Earth());
    phim_.setZero();
    dvbm_.setZero();
    phim_prev_.setZero();
    dvbm_prev_.setZero();
    wib_.setZero();
    wib_prev_.setZero();
    wib_middle_.setZero();
    fb_.setZero();
    fb_prev_.setZero();
    fb_middle_.setZero();
    pos_middle_.setZero();
    vn_middle_.setZero();
    q_middle_ = q_;
    cone_scull_coeff_ <<    2/3,0,0,0,0,
                            9/20,27/20,0,0,0,
                            54/105,92/105,214/105,0,0,
                            250/504,525/504,650/504,1375/504,0,
                            2315/4620,4558/4620,7296/4620,7834/4620,15797/4620;
}

HPSINS::HPSINS(const V3d& att, const V3d& vn, const V3d& pos, const double ts, const int num_samples):
    SINS(att,vn,pos,ts),
    num_samples_(num_samples){
    eth_ = std::unique_ptr<Earth>(new Earth(pos,vn));
    imus_.clear();
    phim_.setZero();
    dvbm_.setZero();
    phim_prev_.setZero();
    dvbm_prev_.setZero();
    wib_.setZero();
    wib_prev_.setZero();
    wib_middle_.setZero();
    fb_.setZero();
    fb_prev_.setZero();
    fb_middle_.setZero();
    pos_middle_ = pos_;
    vn_middle_ = vn_;
    q_middle_ = q_;
    cone_scull_coeff_ <<    2/3,0,0,0,0,
                            9/20,27/20,0,0,0,
                            54/105,92/105,214/105,0,0,
                            250/504,525/504,650/504,1375/504,0,
                            2315/4620,4558/4620,7296/4620,7834/4620,15797/4620;
}

void HPSINS::Update(const IMUData& imu){
    current_imu_timestamp_ = imu.Timestamp();
    if (current_imu_timestamp_ > prev_imu_timestamp_){
        if(imus_.size() < num_samples_){
            imus_.emplace_back(imu);
        }
        else{
            imus_.clear();
            imus_.emplace_back(imu);
        }
    }
    if(imus_.size() == num_samples_){
        update_timestamp_ = imus_.back().Timestamp();
        dt_ = update_timestamp_ - pre_update_timestamp_;
        ConeScullCompensation();
        ComputeWibAndFb();
        V3d extrapolated_pos,extrapolated_vn;
        std::tie(extrapolated_pos,extrapolated_vn) = ExtrapolatePosAndVn(dt_/2.0);
        eth_->EarthUpdate(extrapolated_pos,extrapolated_vn);
        UpdateAttitude();
        UpdateVelocity();
        UpdatePosition();
        pre_update_timestamp_ = update_timestamp_;
    }     
    
}

void HPSINS::UpdateAttitude(){
    // Eigen::AngleAxisd phi_n_in = V3d2AngleAxisd(-eth_->Wnin()*dt_);
    // Eigen::AngleAxisd phi_b_ib = V3d2AngleAxisd((phim_ + prev_phim_)/2);
    // Eigen::Quaterniond q_n_in(phi_n_in);
    // Eigen::Quaterniond q_b_ib(phi_b_ib);

    Eigen::Quaterniond q_n_in = RotationVector2Quaternion(-eth_->Wnin()*dt_);
    Eigen::Quaterniond q_b_ib = RotationVector2Quaternion(wib_middle_*dt_);
    q_ = q_n_in*q_*q_b_ib;

    Eigen::Quaterniond q_n_in_middle = RotationVector2Quaternion(-eth_->Wnin()*dt_/2.0);
    Eigen::Quaterniond q_b_ib_middle = RotationVector2Quaternion(wib_middle_*dt_/2.0);
    q_middle_ = q_n_in_middle*q_*q_b_ib_middle;
}

void HPSINS::UpdateVelocity(){
    an_ = q_middle_*fb_middle_ + eth_->Gcc();
    vn_ += an_*dt_;
    vn_middle_ = (vn_prev_ + vn_)/2.0;
}

void HPSINS::UpdatePosition(){
    pos_ += Vn2DeltaPos(vn_middle_,dt_);
}

//TODO ts_ is not correct,should be the time interval between two consecutive imu data
void HPSINS::ConeScullCompensation(){
    if(imus_.size() < num_samples_)
        return;
    
    phim_.setZero();
    dvbm_.setZero();
    V3d cm(0,0,0),sm(0,0,0),wm(0,0,0),vm(0,0,0); //attention initialized to zero
    for (int i = 0; i < num_samples_ -1; i++){
        cm += cone_scull_coeff_(num_samples_-2,i)*imus_.at(i).Gyro()*ts_;
        sm += cone_scull_coeff_(num_samples_-2,i)*imus_.at(i).Acce()*ts_;
    }

    std::for_each(imus_.begin(),imus_.end(),[&wm,this](const IMUData& imu){wm += imu.Gyro()*ts_;});
    std::for_each(imus_.begin(),imus_.end(),[&vm,this](const IMUData& imu){vm += imu.Acce()*ts_;});

    phim_ = wm + cm.cross(imus_.at(num_samples_-1).Gyro()*ts_); //coning error compensation

    dvbm_ += cm.cross(imus_.at(num_samples_-1).Acce()*ts_) + sm.cross(imus_.at(num_samples_-1).Gyro()*ts_); //sculling error compensation
    dvbm_ += 0.5*wm.cross(vm); // rot error compensation
}

void HPSINS::UpdatePrevSINS(){
    q_prev_ = q_;
    vn_prev_ = vn_;
    phim_prev_ = phim_;
    dvbm_prev_ = dvbm_;
    wib_prev_ = wib_;
    fb_prev_ = fb_;
}

const V3d HPSINS::Vn2DeltaPos(const V3d& vn, double dt) const{
    return V3d(vn(1)*dt/eth_->Rmh(), vn(0)*dt/eth_->ClRnh(), vn(2)*dt);
}

void HPSINS::ComputeWibAndFb(){
    wib_ = phim_*dt_;
    fb_ = dvbm_*dt_;
    wib_middle_ = (wib_prev_ + wib_)/2.0;
    fb_middle_ = (fb_prev_ + fb_)/2.0;
}

std::tuple<V3d,V3d> HPSINS::ExtrapolatePosAndVn(double dt){
    V3d extrapolated_vn = vn_ + an_*dt;
    V3d extrapolated_pos = pos_ + Vn2DeltaPos(vn_,dt);
    return std::make_tuple(extrapolated_pos,extrapolated_vn);
}

} // namespace sins