#include <ros/ros.h>
#include "ros_detctor.h"
#include "ros_trainer.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision_bridge");
    ros::NodeHandle n;

    DetectorService s(n);
    TrainService t(n);

    s.start();
    t.start();

    ros::spin();
    return 0;
}
