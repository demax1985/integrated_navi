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

//	ros::spin();
	return 0;
}

