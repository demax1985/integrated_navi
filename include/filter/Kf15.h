#ifndef KF15_H_
#define KF15_H_

#include "filter_base.h"
#include "sins/sins.h"

namespace sins {


class KF15 : public FilterBase
{
public:
    KF15() {}
//    KF15(const Eigen::VectorXd& state, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q);
    KF15(const Eigen::Matrix<double,15,1> &state, const Eigen::Matrix<double,15,15> &P, const Eigen::Matrix<double,15,15> &Q);
    virtual void Predict(double dt);
    virtual void MeasurementUpdate();
private:
    std::shared_ptr<SINS> pSINS;
};
} //namespace sins
#endif