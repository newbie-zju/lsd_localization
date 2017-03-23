#include <lsd_line/Tracker.h>
int main(int argc, char *argv[])
{
	ros::init(argc,argv,"lsd_line_node");
    TRACKER tracker;
    ros::spin();
	return 0;
}

