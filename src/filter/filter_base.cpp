#include "filter/filter_base.h"

FilterBase::FilterBase(const Eigen::VectorXd &state, const Eigen::MatrixXd &P, const Eigen::MatrixXd &Q):
    converged_(false),
    dt_(0.0){
    state_ = state;
    P_ = P;
    Q_ = Q;
    F_.setZero(0,0);
}
