// Copyright 2021 demax
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "../../include/sins/hpsins.h"

#include "filter/kf15.h"

using sins::HPSINS;
using sins::KF15;

std::tuple<V3d, V3d> SetV3dTuple() {
    return std::make_tuple(V3d(1, 2, 3), V3d(4, 5, 6));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    // HPSINS hpsins(V3d(1,2,3),V3d(0.1,0.1,0.1),V3d(1,2,3));
    HPSINS hpsins;
    hpsins.ShowAtt();
    hpsins.ShowVel();
    hpsins.ShowPos();

    V3d v1 = {0, 0, 45/57.3};
    Eigen::Quaterniond q1 = RotationVector2Quaternion(v1);
    std::cout<< "q1 is: "<< q1.w()<< " "<< q1.x()<< " "<< q1.y()<< " "<< q1.z()<< " "<< std::endl;

    V3d tmp = {1, 0, 0};
    std::cout<< tmp<< std::endl;
    V3d tmp1 = q1*tmp;
    std::cout<< tmp1<< std::endl;

    V3d a1, a2;
    a1 = V3d(2, 2, 2);
    std::tie(a1, a2) = SetV3dTuple();
    std::cout<< "a1 is: "<< a1<< std::endl;
    std::cout<< "a2 is: "<< a2<< std::endl;

    // test exp
    Eigen::MatrixXd Ft(3, 3);
    Ft<< 0.2, 0, 0, 0, 0.3, 0, 0, 0, 0.4;
    std::cout<< "Ft is: "<< Ft<< std::endl;
    std::cout<< "exp of Ft is: "<< Ft.exp()<< std::endl;

    std::cout<< "exp of scaler 0.5 is: "<< exp(0.5)<< std::endl;
    // test diagnoal
    Eigen::Matrix<double, 4, 1> vec_4;
    vec_4 << 1, 2, 3, 4;
    Eigen::DiagonalMatrix<double, 4> dia4 = vec_4.asDiagonal();

    Eigen::Matrix<double, 6, 6> mat1;
    mat1.setZero();
    mat1.block<4, 4>(1, 2) = dia4;
    std::cout<< "diag matrix is: "<< mat1<< std::endl;

    // test symmetric positive
    Eigen::Matrix3d Pk_;
    Pk_<< -1, 0.1, -0.3, 0.2, 2, 0.1, 0.2, 0.3, 3;
    std::cout<< "before,Pk is: "<< std::endl<< Pk_<< std::endl;
    for (int i = 0; i < Pk_.rows(); i++)
    for (int j = 0; j < Pk_.cols(); j++) {
        if (Pk_(i, j) < 0) {
            Pk_(i, j) = -Pk_(i, j);
        }
    }
    Eigen::Matrix3d P1 = (Pk_+Pk_.transpose())/2;
    // std::cout<<"after Pk is: "<<std::endl<<(Pk_+Pk_.transpose())/2<<std::endl;
    std::cout<< "after Pk is: "<< std::endl<< P1<< std::endl;
    // std::cout<<"positive Pk is: "<<std::endl<<Pk_<<std::endl;
    // Eigen::Matrix3d pk_t = Pk_.transpose();
    // std::cout<<"transpose of Pk is: "<<std::endl<<pk_t<<std::endl;
    // Eigen::Matrix3d p3 = Pk_ + pk_t;
    // std::cout<<" Pk+pk.tranpose is: "<<std::endl<<p3<<std::endl;
    // Pk_ = 0.5*p3;
    // std::cout<<"after Pk is: "<<std::endl<<Pk_<<std::endl;

    // test kf15
    V3d att = {0.1, 0.1, 100};
    V3d vn = {100, 100, 0};
    V3d pos = {0.6, 2.4, 100};
    double ts = 0.01;
    double n = 1;
    double taug = 3600;
    double taua = 3600;
    std::shared_ptr<HPSINS> psins(new HPSINS(att, vn, pos, ts, n, taug, taua));
    Eigen::Matrix<double, 15, 15> pk;
    Eigen::Matrix<double, 15, 15> qt;
    qt.setZero();
    Eigen::Matrix<double, 15, 1> tmppk;
    tmppk << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    pk = tmppk.asDiagonal();
    Eigen::Matrix<double, 15, 1> state;
    state.setZero();

    std::unique_ptr<KF15> pkf15(new KF15(state, pk, qt, psins));
    Eigen::Matrix<double, 3, 1> ob = {1, 1, 1};
    Eigen::Matrix<double, 3, 15> Hk;
    Hk.setZero();
    Hk(0, 3) = 1; Hk(1, 4) = 1; Hk(2, 5) = 1;
    Eigen::Matrix3d Rk;
    Rk.setZero();
    Rk(0, 0) = 0.01; Rk(1, 1) = 0.01; Rk(2, 2) = 0.01;
    std::cout<< "Rk is: "<< std::endl<< Rk<< std::endl;

    pkf15->Predict(ts);
    pkf15->MeasurementUpdate(ob, Hk, Rk);
    pkf15->Predict(ts);
    pkf15->MeasurementUpdate(ob, Hk, Rk);
    pkf15->Predict(ts);
    pkf15->MeasurementUpdate(ob, Hk, Rk);

// ros::spin();
    return 0;
}
