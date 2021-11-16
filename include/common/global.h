#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <cmath>
#include <Eigen/Core>
const double pi = 3.141592653589793;

struct GLOBAL
{
    const double Re = 6378137;
    const double f = 1.0/298.257;
    const double wie = 7.2921151467e-5;
    const double Rp = (1.0-f)*Re;
    const double e = sqrtf(2*f-f*f);
    const double e2 = e*e;
    const double ep = sqrtf(Re*Re)/Rp;
    const double ep2 = ep*ep;
    const double g0 = 9.7803267714;
    const double mg = 1.0e-3*g0;
    const double ug = 1.0e-6*g0;
    const double deg = pi/180;
    const double min = deg/60;
    const double sec = min/60;
    const double hour = 3600.0;
    const double dps = deg;
    const double dph = dps/hour;
    const double dpsh = deg/sqrtf(hour);
    const double dphpsh = dph/sqrtf(hour);
    
};

// const Eigen::Matrix<double,5,5> cs ;

//     cs <<   2/3,0,0,0,0,
//             9/20,27/20,0,0,0,
//             54/105,92/105,214/105,0,0,
//             250/504,525/504,650/504,1375/504,0,
//             2315/4620,4558/4620,7296/4620,7834/4620,15797/4620;




#endif