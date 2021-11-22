#include "filter/kf15.h"
namespace sins {


KF15::KF15(const Eigen::Matrix<double,15,1> &state, const Eigen::Matrix<double,15,15> &P, const Eigen::Matrix<double,15,15> &Q,
           std::shared_ptr<SINS> sins):FilterBase(state,P,Q){
    pSINS_ = sins;
    Fk_.resize(15,15);
    Fk_.setZero();
}

void KF15::Predict(double dt){
    SetFk(dt);
    state_ = Fk_*state_;
    Pk_ = Fk_*Pk_*Fk_.transpose() + Qt_*dt;
}

void KF15::MeasurementUpdate(const Eigen::VectorXd Zk, const Eigen::MatrixXd Hk, const Eigen::MatrixXd Rk){
    auto Pxy = Pk_*Hk.transpose();
    auto Py0 = Hk*Pxy;
    auto Py = Py0 + Rk;
    auto innovation = Zk - Hk*state_;
    auto Kk = Pxy*Py.inverse();
    std::cout<<"Kk is: "<<std::endl<<Kk<<std::endl;
    state_ += Kk*innovation;
    Pk_ -= Kk*Hk*Pk_;
    SetPkPositiveSymmetric();
}

void KF15::SetFk(double dt){
    pSINS_->SetErrModelMatrix();
    Eigen::Matrix<double,15,15> Ft;
    Ft.setZero();
    Ft.block<3,3>(int(KfErrorState::kPitch),int(KfErrorState::kPitch)) = pSINS_->Maa();
    Ft.block<3,3>(int(KfErrorState::kPitch),int(KfErrorState::kVe)) = pSINS_->Mav();
    Ft.block<3,3>(int(KfErrorState::kPitch),int(KfErrorState::kLat)) = pSINS_->Map();
    Ft.block<3,3>(int(KfErrorState::kPitch),int(KfErrorState::kGbx)) = -pSINS_->GetRotationMatrix();

    Ft.block<3,3>(int(KfErrorState::kVe),int(KfErrorState::kPitch)) = pSINS_->Mva();
    Ft.block<3,3>(int(KfErrorState::kVe),int(KfErrorState::kVe)) = pSINS_->Mvv();
    Ft.block<3,3>(int(KfErrorState::kVe),int(KfErrorState::kLat)) = pSINS_->Mvp();
    Ft.block<3,3>(int(KfErrorState::kVe),int(KfErrorState::kAbx)) = pSINS_->GetRotationMatrix();

    Ft.block<3,3>(int(KfErrorState::kLat),int(KfErrorState::kVe)) = pSINS_->Mpv();
    Ft.block<3,3>(int(KfErrorState::kLat),int(KfErrorState::kLat)) = pSINS_->Mpp();

    Eigen::Matrix<double,6,1> tau;
    const double tauG = pSINS_->TauG();
    const double tauA = pSINS_->TauA();
    tau << -1.0/tauG, -1.0/tauG,-1.0/tauG, -1.0/tauA, -1.0/tauA, -1.0/tauA;
    Eigen::DiagonalMatrix<double,6> tauGA = tau.asDiagonal();
    Ft.block<6,6>(int(KfErrorState::kGbx),int(KfErrorState::kGbx)) = tauGA;
    Ft *= dt;
    Fk_ = Ft.exp();
    std::cout<<"Fk is: "<<std::endl<<Fk_<<std::endl;
}

void KF15::SetPkPositiveSymmetric(){
    for(int i = 0; i < Pk_.rows(); i++)
        for(int j = 0; j < Pk_.cols(); j++){
            if(Pk_(i,j) < 0){
                Pk_(i,j) = -Pk_(i,j);
            }
        }
    //TODO the following code maybe wrong
    Pk_ = (Pk_ + Pk_.transpose())/2.0;
    std::cout<<"Pk is: "<<std::endl<<Pk_<<std::endl;
}

} //namespace sins
