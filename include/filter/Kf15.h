#ifndef KF15_H_
#define KF15_H_

#include "filter_base.h"
#include "sins/sins.h"

namespace sins {


class KF15 : public FilterBase
{
public:
    KF15() {}
    KF15(const Eigen::VectorXd& state, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q);
    virtual void Predict(double dt);
    virtual void MeasurementUpdate();
private:
    SINS* pSINS;
};
} //namespace sins
#endif
