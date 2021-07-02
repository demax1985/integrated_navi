#ifndef FILTER_BASE_H_
#define FILTER_BASE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class FilterBase
{
public:
    FilterBase() {}
    FilterBase(const Eigen::VectorXd& state, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q);
    virtual ~FilterBase() {}
    virtual void Predict(double dt) = 0;
    virtual void MeasurementUpdate() = 0;

protected:
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd F_;
};

#endif
