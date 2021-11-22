#include "filter/filter_base.h"

FilterBase::FilterBase(const Eigen::VectorXd &state, const Eigen::MatrixXd &P, const Eigen::MatrixXd &Q):
    converged_(false),
    dt_(0.0){
    state_ = state;
    Pk_ = P;
    Qt_ = Q;
    Fk_.setZero(0,0);
}
