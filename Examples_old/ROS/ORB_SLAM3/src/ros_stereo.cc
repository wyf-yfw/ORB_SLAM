/**
 * @file Stereo_Inertial.cpp
 * @brief ORB-SLAM3 双目相机 SLAM 系统的 ROS 节点
 *
 * 此文件实现了一个 ROS 节点，用于接收双目相机的图像，并将其传递给 ORB-SLAM3 系统处理。
 * 可以选择是否对图像进行校正（rectification）。
 *
 * ORB-SLAM3 是自由软件，遵循 GNU 通用公共许可证（GPL）。
 * 有关更多详细信息，请参阅 <http://www.gnu.org/licenses/>.
 */

#include<iostream>  // 引入标准输入输出流
#include<algorithm> // 引入标准算法库
#include<fstream>   // 引入文件流操作
#include<chrono>    // 引入时间相关操作

#include<ros/ros.h>  // 引入 ROS 头文件
#include <cv_bridge/cv_bridge.h>  // 引入 cv_bridge，用于 ROS 图像消息与 OpenCV 图像格式之间的转换
#include <message_filters/subscriber.h>  // 引入消息过滤器（用于同步消息）
#include <message_filters/time_synchronizer.h>  // 引入时间同步器
#include <message_filters/sync_policies/approximate_time.h>  // 引入近似时间同步策略

#include<opencv2/core/core.hpp>  // 引入 OpenCV 核心功能

#include"../../../include/System.h"  // 引入 ORB-SLAM3 系统的头文件

using namespace std;

/**
 * @class ImageGrabber
 * @brief 处理双目相机图像的类，并传递给 ORB-SLAM3 系统
 *
 * ImageGrabber 类的主要职责是接收 ROS 格式的左、右相机图像，进行可选的图像校正，然后传递图像数据给 ORB-SLAM3 系统进行处理。
 */
class ImageGrabber
{
public:
    // 构造函数，初始化 ImageGrabber 类
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    // 处理双目相机图像的回调函数
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;  // 指向 ORB-SLAM3 系统的指针
    bool do_rectify;  // 是否对图像进行校正
    cv::Mat M1l,M2l,M1r,M2r;  // 校正映射矩阵
};

/**
 * @brief 主函数，初始化 ROS 节点，并处理输入参数，订阅双目相机图像数据
 */
int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "RGBD");
    ros::start();

    // 检查输入参数是否正确
    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // 创建 SLAM 系统实例，传入词典路径和设置文件路径
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    // 创建 ImageGrabber 实例
    ImageGrabber igb(&SLAM);

    // 解析第三个参数，确定是否进行图像校正
    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    // 如果需要图像校正，加载相机校准参数
    if(igb.do_rectify)
    {
        // 打开相机校准参数文件
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        // 读取左、右相机的校准参数
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        // 读取图像的宽高
        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        // 检查校准参数是否正确
        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        // 使用校准参数计算图像校正映射矩阵
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;  // 创建 ROS 节点句柄

    // 订阅左、右相机的图像话题
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);

    // 设置消息同步策略，确保左、右相机图像同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();  // 进入 ROS 事件循环

    // 关闭 SLAM 系统
    SLAM.Shutdown();

    // 保存相机轨迹
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

/**
 * @brief 处理双目相机图像的回调函数
 * @param msgLeft 左相机的图像消息
 * @param msgRight 右相机的图像消息
 *
 * 该函数从 ROS 消息中提取左、右相机的图像数据，进行可选的图像校正（rectification），
 * 然后将图像传递给 ORB-SLAM3 系统进行处理。
 */
void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // 将 ROS 图像消息转换为 OpenCV 格式
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);  // 转换左相机图像
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);  // 转换右相机图像
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 如果需要校正，则对图像进行校正
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        // 对左相机图像进行校正
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        // 对右相机图像进行校正
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        // 将校正后的图像传递给 ORB-SLAM3 系统
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
}