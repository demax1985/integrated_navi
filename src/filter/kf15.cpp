#include "filter/kf15.h"
namespace sins {


KF15::KF15(const Eigen::Matrix<double,15,1> &state, const Eigen::Matrix<double,15,15> &P, const Eigen::Matrix<double,15,15> &Q,
           std::shared_ptr<SINS> sins):FilterBase(state,P,Q){
    pSINS_ = sins;
    F_.resize(15,15);
    F_.setZero();
}

void KF15::Predict(double dt){
    // V3d fn = pSINS->GetRotationMatrix()*pSINS->GetIMUdata()->Acce();
}

void KF15::MeasurementUpdate(const Eigen::VectorXd Zk, const Eigen::MatrixXd Hk){
    ;
}

void KF15::SetFk(double dt){
    ;
}



} //namespace sins
