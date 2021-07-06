#include "../../include/sins/lpsins.h"
#include <ros/ros.h>

using namespace sins;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");
    LPSINS lpsins(V3d(1,2,3),V3d(0.1,0.1,0.1),V3d(1,2,3),nullptr);
    lpsins.ShowAtt();
    lpsins.ShowVel();
    lpsins.ShowPos();
//	ros::spin();
	return 0;
}

