#include "ros_detctor.h"
#include <vision_bridge/ObjectArray.h>

using namespace ros;

DetectorService::DetectorService(NodeHandle n){
    mNodeHandle = n;
    mDetectorPtr = new Detector();
    mDetectorPtr->setOnStateChangeCallback(this);
}

int DetectorService::start(){

    /**
     *  获取相关参数
     */
    mNodeHandle.param("/vision_bridge/use_depth", _useDepth, false);
    mNodeHandle.param("/vision_bridge/use_color", _useColor, true);
    mNodeHandle.param("/vision_bridge/islazy", _isLazy, false);
    mNodeHandle.param("/vision_bridge/rgb_topic", _rgbTopicName, std::string("/kinect2/qhd/image_color"));
    mNodeHandle.param("/vision_bridge/depth_topic", _depthTopicName, std::string("/kinect2/qhd/image_depth_rect"));
    mNodeHandle.param("/vision_bridge/camera_frame", _cameraFrame, std::string("kinect2_rgb_optical_frame"));

    /**
     *  发布服务
     */
    detectionServer = mNodeHandle.advertiseService(SERVER_NAME, &DetectorService::detectionCallback, this);
    listDetectorServer = mNodeHandle.advertiseService(LIST_DETECTOR_SERVER_NAME, \
                                                      &DetectorService::listDetectorCallBack, this);
    listObjectServer = mNodeHandle.advertiseService(LIST_OBJECT_SERVER_NAME, \
                                                    &DetectorService::listObjectCallBack, this);

    /**
     *  订阅深度图
     */
    if(_useDepth)
        depthImgSub = mNodeHandle.subscribe(_depthTopicName, 1, &DetectorService::depthImgCB, this);

    /**
     *  订阅彩色图
     */
    if(_useColor)
        colorImgSub = mNodeHandle.subscribe(_rgbTopicName, 1, &DetectorService::colorImgCB, this);

    posePub = mNodeHandle.advertise<vision_bridge::ObjectArray>("object_array", 1);

    ROS_INFO("detector service start finish \n");
}


bool DetectorService::detectionCallback(vision_bridge::detection::Request &req, vision_bridge::detection::Response &res){

    if(_isLazy){
        color_ptr = NULL;
        depth_ptr = NULL;
    }

    /**
     *  订阅深度图
     */
    if(_useDepth && _isLazy)
        depthImgSub = mNodeHandle.subscribe(_depthTopicName, 1, &DetectorService::depthImgCB, this);

    if(_useColor && _isLazy)
        colorImgSub = mNodeHandle.subscribe(_rgbTopicName, 1, &DetectorService::colorImgCB, this);

    /**
     *  wait for image
     */
    for(int i = 0; i < 20; i ++){
        if( ( _useColor && color_ptr == NULL) || ( _useDepth && depth_ptr == NULL) ){
            ros::Duration(1.0).sleep();
            continue;
        }
        break;
    }

    if( ( _useColor && color_ptr == NULL) || ( _useDepth && depth_ptr == NULL) ){
        res.result = -1;
        std::cout << "can't no got img" << std::endl;
        return false;
    }

    ENTITY_TYPE type;
    if(req.detectorType == 1)
        type = PYTHON;
    else
        type = CPP;

    mDetectorPtr->setDetector(req.detectorName, req.objectName, type, req.detectorConfig);

    cv::Mat depth;
    cv::Mat color;

    if( _useDepth )
        depth = depth_ptr->image;

    if( _useColor )
        color = color_ptr->image;

    mDetectorPtr->detectionOnce(depth, color);

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

    msg.header.frame_id = _cameraFrame;

    msg.header.stamp.nsec = ros::Time(0).nsec;
    msg.header.stamp.sec = ros::Time(0).sec;

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

    std::cout << "Got depth img" << std::endl;

    try
    {
        depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(_isLazy)
        depthImgSub.shutdown();

}

void DetectorService::colorImgCB(const sensor_msgs::ImageConstPtr& msg){

    std::cout << "Got color img" << std::endl;

    try
    {
        color_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(_isLazy)
        colorImgSub.shutdown();
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
