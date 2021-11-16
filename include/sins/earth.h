#ifndef SINS_EARTH_H
#define SINS_EARTH_H

#include <Eigen/Core>
#include <Eigen/Dense>
using V3d = Eigen::Vector3d;
class Earth
{
private:
    double Re_;
    double e2_;
    double sl_;
    double cl_;
    double tl_;
    double sl2_;
    double rnh_;
    double cl_rnh_;
    double rmh_;
    double wie_;
    double g0_;
    double g_;
    
    V3d pos_;
    V3d vn_;
    V3d wnie_;
    V3d wnen_;
    V3d wnin_;
    V3d wnien_; //wnien = wnie + wnin
    V3d gn_;
    V3d gcc_;

    
public:
    Earth(/* args */);
    Earth(const V3d& pos, const V3d& vn);
    void EarthUpdate(const V3d& pos, const V3d& vn);
};












#endif