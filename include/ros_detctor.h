#ifndef __ROS_DETECTOR_H__
#define __ROS_DETECTOR_H__

#include "trainer.h"
#include "detector.h"
#include "ros/ros.h"
#include "vision_bridge/detection.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace hirop_vision;


#define SERVER_NAME "detection"
class DetectorService:public DetectStateListener{

public:
    DetectorService(ros::NodeHandle n);

    void onDetectDone(std::string detector, int ret, std::vector<pose> p);

    int start();

    int stop();

private:
    bool detectionCallback(vision_bridge::detection::Request &req, vision_bridge::detection::Response &res);

    void depthImgCB(const sensor_msgs::ImageConstPtr& msg);

    void colorImgCB(const sensor_msgs::ImageConstPtr& msg);

private:
    Detector *mDetectorPtr;

    ros::ServiceServer detectionServer;

    ros::Subscriber colorImgSub;
    ros::Subscriber depthImgSub;

    ros::NodeHandle mNodeHandle;

    cv_bridge::CvImagePtr depth_ptr;

    cv_bridge::CvImagePtr color_ptr;

    ros::Publisher posePub;
};

#endif
