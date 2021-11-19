#include "../../include/sins/hpsins.h"
#include <ros/ros.h>

using namespace sins;
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
//	ros::spin();
	return 0;
}

