/**
 * @file Mono_Inertial.cpp
 * @brief ORB-SLAM3 单目惯性 SLAM 系统的 ROS 节点
 *
 * 此文件实现了一个 ROS 节点，使用单目相机和 IMU 数据来进行 SLAM（同步图像和 IMU）。
 *
 * @copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * @copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 是自由软件：您可以在 GNU 通用公共许可证第 3 版的条款下重新分发和/或修改它。
 */

#include<iostream>  // 引入输入输出流
#include<algorithm> // 引入标准算法库
#include<fstream>   // 引入文件流
#include<chrono>    // 引入时间库
#include<vector>    // 引入向量容器
#include<queue>     // 引入队列容器
#include<thread>    // 引入线程库
#include<mutex>     // 引入互斥量

#include<ros/ros.h>  // 引入 ROS 头文件
#include<cv_bridge/cv_bridge.h>  // 引入 cv_bridge，用于转换 ROS 图像消息
#include<sensor_msgs/Imu.h>  // 引入 IMU 消息格式

#include<opencv2/core/core.hpp>  // 引入 OpenCV 核心功能

#include"../../../include/System.h"  // 引入 ORB-SLAM3 系统头文件
#include"../include/ImuTypes.h"  // 引入 IMU 类型

using namespace std;

/**
 * @class ImuGrabber
 * @brief 从 IMU 获取数据并存入队列的类
 */
class ImuGrabber
{
public:
    ImuGrabber(){};  // 构造函数
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);  // 处理 IMU 数据的回调函数

    queue<sensor_msgs::ImuConstPtr> imuBuf;  // IMU 数据缓冲区（队列）
    std::mutex mBufMutex;  // 用于保护 imuBuf 的互斥量
};

/**
 * @class ImageGrabber
 * @brief 从相机获取图像并与 IMU 数据进行同步的类
 */
class ImageGrabber
{
public:
    // 构造函数，初始化 ImageGrabber 类
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);  // 图像获取的回调函数
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);  // 将 ROS 图像消息转换为 OpenCV 格式
    void SyncWithImu();  // 与 IMU 数据同步

    queue<sensor_msgs::ImageConstPtr> img0Buf;  // 图像缓冲区（队列）
    std::mutex mBufMutex;  // 用于保护 img0Buf 的互斥量

    ORB_SLAM3::System* mpSLAM;  // 指向 SLAM 系统的指针
    ImuGrabber *mpImuGb;  // 指向 IMU 数据获取器的指针

    const bool mbClahe;  // 是否使用 CLAHE 进行图像均衡化
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));  // CLAHE 图像增强器
};


int main(int argc, char **argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  bool bEqual = false;  // 用于控制是否进行图像均衡化
  // 检查命令行参数数量
  if(argc < 3 || argc > 4)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  // 检查是否需要图像均衡化
  if(argc == 4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // 创建 SLAM 系统，初始化所有线程，准备处理帧
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

  ImuGrabber imugb;  // 创建 IMU 数据抓取器
  ImageGrabber igb(&SLAM, &imugb, bEqual);  // 创建图像抓取器

  // 订阅 IMU 话题，处理 IMU 数据
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  // 订阅相机图像话题，处理图像数据
  ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage, &igb);

  // 创建线程用于同步 IMU 和图像数据
  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  ros::spin();  // 进入 ROS 事件循环

  return 0;
}

/**
 * @brief 处理图像消息的回调函数
 * @param img_msg ROS 图像消息
 */
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // 上锁保护图像缓冲区
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();  // 清除缓冲区中的旧图像
  img0Buf.push(img_msg);  // 将新图像放入缓冲区
  mBufMutex.unlock();  // 解锁
}

/**
 * @brief 将 ROS 图像消息转换为 OpenCV 格式
 * @param img_msg ROS 图像消息
 * @return 转换后的 OpenCV Mat 图像
 */
cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // 使用 cv_bridge 将 ROS 图像转换为 OpenCV Mat 格式
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);  // 转换为单通道灰度图
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // 返回图像副本
  if(cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();  // 返回克隆的图像
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

/**
 * @brief 同步图像和 IMU 数据的函数
 *
 * 不断检查图像和 IMU 数据的时间戳，确保它们同步并传递给 SLAM 系统进行处理。
 */
void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    // 检查图像和 IMU 缓冲区是否都有数据
    if (!img0Buf.empty() && !mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec();  // 获取图像时间戳
      // 如果图像时间戳比 IMU 数据晚，跳过此帧
      if(tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      // 处理图像数据
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());  // 获取图像
      img0Buf.pop();  // 移除已处理的图像
      this->mBufMutex.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;  // 用于存储 IMU 测量数据
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // 从缓冲区加载 IMU 测量数据
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();  // 获取 IMU 时间戳
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);  // 加速度
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);  // 角速度
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));  // 将 IMU 数据存入向量
          mpImuGb->imuBuf.pop();  // 移除已处理的 IMU 数据
        }
      }
      mpImuGb->mBufMutex.unlock();

      // 如果启用了 CLAHE，进行图像均衡化
      if(mbClahe)
        mClahe->apply(im, im);

      // 将图像和 IMU 数据传递给 SLAM 系统
      mpSLAM->TrackMonocular(im, tIm, vImuMeas);
    }

    // 线程休眠 1 毫秒
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

/**
 * @brief 处理 IMU 消息的回调函数
 * @param imu_msg ROS IMU 消息
 */
void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  // 上锁保护 IMU 缓冲区
  mBufMutex.lock();
  imuBuf.push(imu_msg);  // 将 IMU 数据放入缓冲区
  mBufMutex.unlock();  // 解锁
  return;
}
