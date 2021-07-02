#include "filter/filter_base.h"

FilterBase::FilterBase(const Eigen::VectorXd &state, const Eigen::MatrixXd &P, const Eigen::MatrixXd &Q){
    state_ = state;
    P_ = P;
    Q_ = Q;
}
