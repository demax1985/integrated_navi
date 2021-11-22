#include "../../include/sins/hpsins.h"
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

using namespace sins;

std::tuple<V3d,V3d> SetV3dTuple(){
    return std::make_tuple(V3d(1,2,3),V3d(4,5,6));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");
    // HPSINS hpsins(V3d(1,2,3),V3d(0.1,0.1,0.1),V3d(1,2,3));
    HPSINS hpsins;
    hpsins.ShowAtt();
    hpsins.ShowVel();
    hpsins.ShowPos();

    V3d v1 = {0,0,45/57.3};
    Eigen::Quaterniond q1 = RotationVector2Quaternion(v1);
    std::cout<<"q1 is: "<<q1.w()<<" "<<q1.x()<<" "<<q1.y()<<" "<<q1.z()<<" "<<std::endl;

    V3d tmp = {1,0,0};
    std::cout<<tmp<<std::endl;
    V3d tmp1 = q1*tmp;
    std::cout<<tmp1<<std::endl;

    V3d a1,a2;
    a1 = V3d(2,2,2);
    std::tie(a1,a2) = SetV3dTuple();
    std::cout<<"a1 is: "<<a1<<std::endl;
    std::cout<<"a2 is: "<<a2<<std::endl;

    //test exp
    Eigen::MatrixXd Ft(3,3);
    Ft<<0.2,0,0,0,0.3,0,0,0,0.4;
    std::cout<<"Ft is: "<<Ft<<std::endl;
    std::cout<<"exp of Ft is: "<<Ft.exp()<<std::endl;

    std::cout<<"exp of scaler 0.5 is: "<<exp(0.5)<<std::endl;
    //test diagnoal
    Eigen::Matrix<double,4,1> vec_4;
    vec_4 << 1,2,3,4;
    Eigen::DiagonalMatrix<double,4> dia4 = vec_4.asDiagonal();

    Eigen::Matrix<double,6,6> mat1;
    mat1.setZero();
    mat1.block<4,4>(1,2) = dia4;
    std::cout<<"diag matrix is: "<<mat1<<std::endl;
//	ros::spin();
	return 0;
}

