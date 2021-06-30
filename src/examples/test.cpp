#include "../../include/sins/lpsins.h"
#include <ros/ros.h>

using namespace sins;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");
	LPSINS lpsins;
	ros::spin();
	return 0;
}

