/**
 * @file Mono.cpp
 * @brief ORB-SLAM3 单目视觉 SLAM 系统的 ROS 节点
 *
 * 此文件实现了一个 ROS 节点，用于从相机获取图像并传递给 ORB-SLAM3 系统进行处理。
 *
 * @copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * @copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 是自由软件：您可以在 GNU 通用公共许可证第 3 版的条款下重新分发和/或修改它。
 */

#include<iostream>  // 引入输入输出流
#include<algorithm> // 引入算法库
#include<fstream>   // 引入文件流
#include<chrono>    // 引入时间库

#include<ros/ros.h>  // 引入 ROS 头文件
#include <cv_bridge/cv_bridge.h>  // 引入 CV Bridge，用于转换 ROS 图像消息

#include<opencv2/core/core.hpp>  // 引入 OpenCV 核心功能

#include"../../../include/System.h"  // 引入 ORB-SLAM3 系统头文件

using namespace std;  // 使用标准命名空间

/**
 * @class ImageGrabber
 * @brief 从相机获取图像并将其传递给 SLAM 系统的类
 */
class ImageGrabber
{
public:
    // 构造函数，初始化 ImageGrabber 类
    ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM) {}

    // 从 ROS 消息抓取图像
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;  // 指向 SLAM 系统的指针
};

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "Mono");
    ros::start();

    // 检查命令行参数数量
    if (argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();  // 关闭 ROS
        return 1;
    }

    // 创建 SLAM 系统，初始化所有线程，准备处理帧
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);  // 创建图像抓取器

    ros::NodeHandle nodeHandler;  // 创建 ROS 节点句柄
    // 订阅相机图像话题，注册图像抓取回调函数
    ros::Subscriber sub = nodeHandler.subscribe("/camera/color/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();  // 进入 ROS 事件循环

    // 停止所有线程
    SLAM.Shutdown();

    // 保存相机轨迹到文件
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();  // 关闭 ROS

    return 0;  // 返回 0，表示程序正常结束
}

/**
 * @brief 从 ROS 消息中抓取图像并传递给 SLAM 系统进行处理
 * @param msg ROS 图像消息
 */
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // 将 ROS 图像消息复制到 cv::Mat 格式
    cv_bridge::CvImageConstPtr cv_ptr;  // 创建 cv_bridge 指针
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);  // 转换图像
    }
    catch (cv_bridge::Exception& e)  // 捕获转换异常
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());  // 打印错误信息
        return;  // 退出函数
    }

    // 将图像和时间戳传递给 SLAM 系统进行处理
    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}
