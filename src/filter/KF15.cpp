#include "filter/Kf15.h"
namespace sins {


KF15::KF15(const Eigen::Matrix<double,15,1> &state, const Eigen::Matrix<double,15,15> &P, const Eigen::Matrix<double,15,15> &Q):FilterBase(state,P,Q){
    pSINS = std::shared_ptr<SINS>();
    F_.resize(15,15);
    F_.setZero();

}

void KF15::Predict(double dt){
    // V3d fn = pSINS->GetRotationMatrix()*pSINS->GetIMUdata()->Acce();
}

void KF15::MeasurementUpdate(){
    ;
}

} //namespace sins
