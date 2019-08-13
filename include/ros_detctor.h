#ifndef __ROS_DETECTOR_H__
#define __ROS_DETECTOR_H__

#include <vision/detector.h>
#include <ros/ros.h>

#include <vision_bridge/detection.h>
#include <vision_bridge/listDetector.h>
#include <vision_bridge/listObject.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace hirop_vision;


#define SERVER_NAME "detection"
#define LIST_DETECTOR_SERVER_NAME "list_detector"
#define LIST_OBJECT_SERVER_NAME "list_object"

class DetectorService:public DetectStateListener{

public:
    /**
     * @brief   DetectorService 检测服务的构造函数
     * @param[in] n 当前ROS节点的句柄
     */
    DetectorService(ros::NodeHandle n);

    /**
     * @brief onDetectDone  当检测完成后的回调函数
     * @param detector      检测器的名称
     * @param ret           检测器的返回值
     * @param p             检测到物体的位姿列表
     */
    void onDetectDone(std::string detector, int ret, std::vector<pose> p);

    /**
     * @brief start     启动检测服务
     * @return  0 成功 -1 失败
     */
    int start();

    /**
     * @brief stop      停止检测服务
     * @return      0 成功 -1 失败
     */
    int stop();

private:
    /**
     * @brief detectionCallback 检测服务的回调函数
     * @param req   检测的参数
     * @param res   检测的结果（检测器成功开始检测后返回0，检测器开启检测县城失败后返回-1）
     * @return  当检测器正常开始检测时返回ture 当检测器无法启动时返回false
     */
    bool detectionCallback(vision_bridge::detection::Request &req, vision_bridge::detection::Response &res);

    /**
     * @brief listDetectorCallBack  列出系统中可用的检测器
     * @param req   空
     * @param res   返回检测器的列表
     * @return      永远为true
     */
    bool listDetectorCallBack(vision_bridge::listDetector::Request &req, vision_bridge::listDetector::Response &res);

    /**
     * @brief listObjectCallBack  列出系统中可识别的物体
     * @param req   空
     * @param res   系统中可识别的物体列表
     * @return      永远为true
     */
    bool listObjectCallBack(vision_bridge::listObject::Request &req, vision_bridge::listObject::Response &res);

    /**
     * @brief depthImgCB    接受到深度图后的回调函数
     * @param msg           深度图
     */
    void depthImgCB(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief colorImgCB    接受到彩色图后的回调函数
     * @param msg           彩色图
     */
    void colorImgCB(const sensor_msgs::ImageConstPtr& msg);

private:
    /**
     * @brief mDetectorPtr  保存检测器的实例指针
     */
    Detector *mDetectorPtr;

    /**
     * @brief 各项服务的实例
     */
    ros::ServiceServer detectionServer;
    ros::ServiceServer listDetectorServer;
    ros::ServiceServer listObjectServer;

    /**
     * @brief 深度图和彩色图的监听者
     */
    ros::Subscriber colorImgSub;
    ros::Subscriber depthImgSub;

    /**
     * @brief mNodeHandle   ROS节点
     */
    ros::NodeHandle mNodeHandle;

    /**
     * @brief depth_ptr    全局的图像指针
     */
    cv_bridge::CvImagePtr depth_ptr;
    cv_bridge::CvImagePtr color_ptr;

    /**
     * @brief posePub   物体位姿发布器
     */
    ros::Publisher posePub;

    bool _useDepth;
    bool _useColor;

    std::string _rgbTopicName;
    std::string _depthTopicName;
    std::string _cameraFrame;


};

#endif
