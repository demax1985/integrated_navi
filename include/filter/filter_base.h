#ifndef FILTER_FILTER_BASE_H_
#define FILTER_FILTER_BASE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class FilterBase
{
public:
    FilterBase() {}
    FilterBase(const Eigen::VectorXd& state, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q);
    virtual ~FilterBase() {}
    virtual void SetFk(double dt) = 0;
    virtual void Predict(double dt) = 0;
    virtual void MeasurementUpdate(const Eigen::VectorXd Zk, const Eigen::MatrixXd Hk) = 0;

protected:
    bool converged_;
    double dt_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd Pk_;
    Eigen::MatrixXd Qk_;
    Eigen::MatrixXd Fk_;
};

#endif
