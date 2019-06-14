#include "ros_detctor.h"
#include "vision_bridge/ObjectArray.h"

using namespace ros;

DetectorService::DetectorService(NodeHandle n){
    mNodeHandle = n;
    mDetectorPtr = new Detector();
    mDetectorPtr->setOnStateChangeCallback(this);
}

int DetectorService::start(){
    detectionServer = mNodeHandle.advertiseService(SERVER_NAME, &DetectorService::detectionCallback, this);
    listDetectorServer = mNodeHandle.advertiseService(LIST_DETECTOR_SERVER_NAME, \
                                                      &DetectorService::listDetectorCallBack, this);
    listObjectServer = mNodeHandle.advertiseService(LIST_OBJECT_SERVER_NAME, \
                                                    &DetectorService::listObjectCallBack, this);
    depthImgSub = mNodeHandle.subscribe("/kinect2/qhd/image_depth_rect", 1, &DetectorService::depthImgCB, this);
    colorImgSub = mNodeHandle.subscribe("/kinect2/qhd/image_color", 1, &DetectorService::colorImgCB, this);
    posePub = mNodeHandle.advertise<vision_bridge::ObjectArray>("object_array", 1);
    ROS_INFO("detector service start finish \n");
}


bool DetectorService::detectionCallback(vision_bridge::detection::Request &req, vision_bridge::detection::Response &res){

    if(color_ptr == NULL || depth_ptr == NULL){
        res.result = -1;
        return false;
    }

    ENTITY_TYPE type;
    if(req.detectorType == 1)
        type = PYTHON;
    else
        type = CPP;

    mDetectorPtr->setDetector(req.detectorName, req.objectName, type, req.detectorConfig);
    mDetectorPtr->detectionOnce(depth_ptr->image, color_ptr->image);

    return true;
}

void DetectorService::onDetectDone(std::string detector, int ret, std::vector<pose> p){

    ROS_INFO("detector was %s", detector.c_str());
    ROS_INFO("ret was %d", ret);
    if(p.empty())
        return ;
    ROS_INFO("x = %lf, y = %lf, z = %lf", p[0].position.x, p[0].position.y, p[0].position.z);

    vision_bridge::ObjectArray msg;
    ros::Time now;

    msg.header.frame_id = "kinect2_rgb_optical_frame";

    msg.header.stamp.nsec = now.now().nsec;
    msg.header.stamp.sec = now.now().sec;

    for(int i =  0; i < p.size(); i ++){
        vision_bridge::ObjectInfo objectTmp;

        objectTmp.name = p[i].objectName.c_str();
        objectTmp.detector = detector.c_str();

        objectTmp.pose.header = msg.header;

        objectTmp.pose.pose.orientation.x = p[i].quaternion.x;
        objectTmp.pose.pose.orientation.y = p[i].quaternion.y;
        objectTmp.pose.pose.orientation.z = p[i].quaternion.z;
        objectTmp.pose.pose.orientation.w = p[i].quaternion.w;

        objectTmp.pose.pose.position.x = p[i].position.x;
        objectTmp.pose.pose.position.y = p[i].position.y;
        objectTmp.pose.pose.position.z = p[i].position.z;

        msg.objects.push_back(objectTmp);
    }

    posePub.publish(msg);

}

void DetectorService::depthImgCB(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void DetectorService::colorImgCB(const sensor_msgs::ImageConstPtr& msg){

    try
    {
        color_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

bool DetectorService::listDetectorCallBack(vision_bridge::listDetector::Request &req,
                                           vision_bridge::listDetector::Response &res){


    std::vector<std::string> list;
    mDetectorPtr->getDetectorList(list);

    res.detectorList = list;

    return true;
}

bool DetectorService::listObjectCallBack(vision_bridge::listObject::Request &req,
                                         vision_bridge::listObject::Response &res){

    std::vector<std::string> list;
    mDetectorPtr->getObjectList(req.detectorName, list);
    res.objectList = list;
    return true;

}
