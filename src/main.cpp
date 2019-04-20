#include <ros/ros.h>
#include "ros_detctor.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision_bridge");
    ros::NodeHandle n;

    DetectorService s(n);
    s.start();
    ros::spin();
    return 0;
}
