#include "sins/hpsins.h"

namespace sins{

HPSINS::HPSINS():SINS(){
    imus_.clear();
    eth_ = std::unique_ptr<Earth>(new Earth());
    phim_.setZero();
    dvbm_.setZero();
    prev_phim_.setZero();
    prev_dvbm_.setZero();
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
    prev_phim_.setZero();
    prev_dvbm_.setZero();
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
    ConeScullCompensation();
    


    prev_imu_timestamp_ = current_imu_timestamp_;
}

void HPSINS::UpdateAttitude(){
    // Eigen::AngleAxisd phi_n_in(eth_->Wnin());
    // Eigen::Quaterniond qn_in(eth_->Wnin());

}

void HPSINS::UpdateVelocity(){

}

void HPSINS::UpdatePosition(){

}

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









} // namespace sins