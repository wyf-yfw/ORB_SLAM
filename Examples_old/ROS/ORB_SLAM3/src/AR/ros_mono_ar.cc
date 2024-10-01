/**
* @file Mono.cc
* @brief ORB-SLAM3的增强现实演示，使用单目相机进行SLAM并结合AR展示虚拟物体
*
* 本代码使用单目相机来获取图像，通过ORB-SLAM3进行相机位姿跟踪，并使用增强现实技术（AR）展示虚拟物体。
* 使用ROS接收图像话题，通过OpenCV处理图像，并将结果传递给ViewerAR类来渲染AR场景。
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <Eigen/Dense>  // Eigen库，用于矩阵操作
#include <opencv2/core/eigen.hpp>  // OpenCV和Eigen矩阵的转换工具

#include<ros/ros.h>  // ROS标准头文件
#include <cv_bridge/cv_bridge.h>  // OpenCV和ROS图像消息之间的转换工具

#include<opencv2/core/core.hpp>  // OpenCV核心库
#include<opencv2/imgproc/imgproc.hpp>  // OpenCV图像处理库

#include"../../../include/System.h"  // ORB-SLAM3系统类
#include"ViewerAR.h"  // ViewerAR类，用于显示AR效果

using namespace std;

ORB_SLAM3::ViewerAR viewerAR;  // ViewerAR对象，用于增强现实的渲染
bool bRGB = true;  // 相机是否使用RGB格式

cv::Mat K;  // 相机内参矩阵
cv::Mat DistCoef;  // 相机畸变系数

// 图像获取类，处理ROS消息并传递给ORB-SLAM3进行处理
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}  // 构造函数，初始化ORB-SLAM3系统指针

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);  // 回调函数，用于处理ROS图像消息

    ORB_SLAM3::System* mpSLAM;  // ORB-SLAM3系统指针，用于调用SLAM功能
};

int main(int argc, char **argv)
{
    // ROS节点初始化，节点名为 "Mono"
    ros::init(argc, argv, "Mono");
    ros::start();

    // 检查输入参数，确保提供了词汇文件和设置文件路径
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // 创建ORB-SLAM3系统，模式为单目SLAM（MONOCULAR），不使用Viewer
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);

    // 打印AR演示提示信息
    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;

    // 将SLAM系统传递给ViewerAR类
    viewerAR.SetSLAM(&SLAM);

    // 创建图像获取对象
    ImageGrabber igb(&SLAM);

    // ROS节点句柄，用于创建订阅器
    ros::NodeHandle nodeHandler;
    // 订阅相机图像话题，使用回调函数处理图像
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    // 从设置文件中读取相机参数
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);  // 读取是否为RGB格式
    float fps = fSettings["Camera.fps"];  // 读取相机帧率
    viewerAR.SetFPS(fps);  // 设置Viewer的帧率

    // 读取相机内参
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    viewerAR.SetCameraCalibration(fx, fy, cx, cy);  // 设置Viewer的相机校准参数

    // 初始化相机内参矩阵K
    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    // 初始化畸变系数矩阵DistCoef
    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];  // 如果有第五个畸变系数k3，则扩展畸变矩阵
    if(k3 != 0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    // 启动ViewerAR线程，负责渲染AR效果
    thread tViewer = thread(&ORB_SLAM3::ViewerAR::Run,&viewerAR);

    // ROS事件循环，等待图像消息到来
    ros::spin();

    // 停止SLAM系统
    SLAM.Shutdown();

    // 保存关键帧轨迹
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // 关闭ROS
    ros::shutdown();

    return 0;
}

/**
* @brief 图像获取回调函数，将ROS图像消息转换为OpenCV格式并传递给ORB-SLAM3进行处理
* @param msg ROS图像消息
*/
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 克隆图像
    cv::Mat im = cv_ptr->image.clone();
    cv::Mat imu;
    cv::Mat Tcw;

    // 使用ORB-SLAM3系统跟踪单目图像，返回相机位姿Tcw（Sophus::SE3f类型）
    Sophus::SE3f Tcw_SE3f = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    // 将Eigen矩阵转换为OpenCV矩阵
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, Tcw);

    // 获取跟踪状态
    int state = mpSLAM->GetTrackingState();

    // 获取跟踪的地图点和关键点
    vector<ORB_SLAM3::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

    // 对图像进行去畸变处理
    cv::undistort(im, imu, K, DistCoef);

    // 根据是否为RGB图像，设置AR显示的图像和位姿信息
    if(bRGB)
        viewerAR.SetImagePose(imu, Tcw, state, vKeys, vMPs);
    else
    {
        // 如果不是RGB图像，则进行颜色空间转换
        cv::cvtColor(imu, imu, CV_RGB2BGR);
        viewerAR.SetImagePose(imu, Tcw, state, vKeys, vMPs);
    }
}
