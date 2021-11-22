#ifndef FILTER_KF15_H_
#define FILTER_KF15_H_

#include "filter_base.h"
#include <memory>
#include <unsupported/Eigen/MatrixFunctions>
#include "sins/sins.h"
#include "sins/hpsins.h"


namespace sins {


class KF15 : public FilterBase
{
public:
    enum class KfErrorState{kPitch,kRoll,Kyaw,kVe,kVn,kVu,kLat,kLon,kAlt,kGbx,kGby,kGbz,kAbx,kAby,kAbz};
    KF15() {}
//    KF15(const Eigen::VectorXd& state, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q);
    KF15(const Eigen::Matrix<double,15,1> &state, const Eigen::Matrix<double,15,15> &P, const Eigen::Matrix<double,15,15> &Q,
         std::shared_ptr<SINS> sins);
    virtual void SetFk(double dt) override;
    virtual void Predict(double dt) override;
    virtual void MeasurementUpdate(const Eigen::VectorXd Zk, const Eigen::MatrixXd Hk) override;
private:
    std::shared_ptr<SINS> pSINS_;
};
} //namespace sins
#endif
