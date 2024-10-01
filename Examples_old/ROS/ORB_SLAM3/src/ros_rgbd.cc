/**
* @file RGBD.cc
* @brief ROS节点用于ORB-SLAM3 RGB-D模式的图像处理与SLAM跟踪
*
* 本程序使用ROS（Robot Operating System）接收RGB图像和深度图像，并将其传递给ORB-SLAM3进行SLAM处理。
* 该节点主要功能包括：图像同步、SLAM初始化、图像数据的接收与传递、关键帧轨迹的保存。
*
* @note 在运行前需启动与RGB-D传感器相关的ROS节点，如RealSense或其他兼容的RGB-D相机节点。
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* 本程序是ORB-SLAM3开源项目的一部分，遵循GNU通用公共许可证（GPL）v3或更高版本发布。
* ORB-SLAM3是免费的开源软件，您可以根据许可证的条款自由地重新分发和修改它。
* 详见 http://www.gnu.org/licenses/ 获取更多许可信息。
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h> // ROS标准头文件
#include <cv_bridge/cv_bridge.h> // OpenCV和ROS图像消息的转换工具
#include <message_filters/subscriber.h> // 消息过滤器，用于图像消息的同步
#include <message_filters/time_synchronizer.h> // 时间同步器，用于对多话题消息进行同步
#include <message_filters/sync_policies/approximate_time.h> // 近似时间同步策略

#include<opencv2/core/core.hpp> // OpenCV核心库

#include"../../../include/System.h" // ORB-SLAM3主系统类定义

using namespace std;

// 定义图像获取类
class ImageGrabber
{
public:
    /**
    * @brief 构造函数，初始化ORB-SLAM3系统指针
    * @param pSLAM ORB-SLAM3系统指针，用于传递图像数据进行SLAM处理
    */
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    /**
    * @brief RGB-D图像数据同步接收回调函数
    * @param msgRGB RGB图像的ROS消息
    * @param msgD 深度图像的ROS消息
    */
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

    // ORB-SLAM3系统指针
    ORB_SLAM3::System* mpSLAM;
};

// 主程序入口
int main(int argc, char **argv)
{
    // ROS初始化，节点名称为 "RGBD"
    ros::init(argc, argv, "RGBD");
    // 启动ROS节点
    ros::start();

    // 检查命令行参数是否正确（需要传递词汇文件和设置文件的路径）
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // 创建ORB-SLAM3系统对象，指定词汇表路径、设置文件路径、传感器类型（RGB-D模式）和使用Viewer界面
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    // 创建图像获取类的对象，并传入ORB-SLAM3系统指针
    ImageGrabber igb(&SLAM);

    // 创建ROS节点句柄
    ros::NodeHandle nh;

    // 创建消息过滤器订阅器，订阅RGB和深度图像话题
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 100);

    // 定义近似时间同步策略
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    // 创建时间同步器，设置队列长度为10，关联两个订阅器
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);

    // 注册图像同步回调函数，使用Boost绑定方法将GrabRGBD与图像同步器关联
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    // 进入ROS事件循环，等待图像消息的到来
    ros::spin();

    // 在节点关闭前，停止ORB-SLAM3系统所有线程
    SLAM.Shutdown();

    // 保存相机的关键帧轨迹到文件
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // 关闭ROS
    ros::shutdown();

    return 0;
}

/**
* @brief 图像同步回调函数，将ROS图像消息转换为OpenCV格式，并传递给ORB-SLAM3进行处理
* @param msgRGB ROS消息格式的RGB图像
* @param msgD ROS消息格式的深度图像
*/
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    // 使用cv_bridge将ROS的图像消息转换为OpenCV格式（cv::Mat）
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 同样将深度图像转换为OpenCV格式
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 将RGB图像和深度图像传递给ORB-SLAM3系统进行跟踪，输入为cv::Mat格式图像以及时间戳
    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
}
