/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;
// IMU 数据抓取器类，用于接收和缓存 IMU 数据
class ImuGrabber
{
public:
  // 默认构造函数
  ImuGrabber(){};

  // 接收 IMU 数据并将其保存到队列中
  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

  // IMU 数据队列，用于保存接收到的 IMU 消息
  std::queue<sensor_msgs::ImuConstPtr> imuBuf;

  // 互斥锁，用于保护 IMU 数据队列的多线程访问
  std::mutex mBufMutex;
};

// 图像抓取器类，用于接收左右相机的图像，并与 IMU 数据同步
class ImageGrabber
{
public:
  // 构造函数，初始化 ORB-SLAM 系统指针、IMU 抓取器、是否进行图像校正和 CLAHE（对比度受限自适应直方图均衡化）
  ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe)
      : mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe) {}

  // 接收左侧相机图像的回调函数
  void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);

  // 接收右侧相机图像的回调函数
  void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);

  // 将 ROS 的图像消息转换为 OpenCV 格式的图像
  cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

  // 与 IMU 数据进行同步
  void SyncWithImu();

  // 左相机图像队列，用于保存接收到的左相机图像
  std::queue<sensor_msgs::ImageConstPtr> imgLeftBuf;

  // 右相机图像队列，用于保存接收到的右相机图像
  std::queue<sensor_msgs::ImageConstPtr> imgRightBuf;

  // 互斥锁，用于保护左相机图像队列的多线程访问
  std::mutex mBufMutexLeft;

  // 互斥锁，用于保护右相机图像队列的多线程访问
  std::mutex mBufMutexRight;

  // ORB-SLAM 系统指针，用于调用 SLAM 系统的功能
  ORB_SLAM3::System* mpSLAM;

  // IMU 数据抓取器指针，用于同步 IMU 数据
  ImuGrabber *mpImuGb;

  // 标志是否进行图像校正
  const bool do_rectify;

  // 用于图像校正的映射矩阵
  cv::Mat M1l, M2l, M1r, M2r;

  // 标志是否使用 CLAHE（对比度受限自适应直方图均衡化）
  const bool mbClahe;

  // CLAHE 对象，用于图像增强
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


int main(int argc, char **argv)
{
    // 初始化 ROS 节点 "Stereo_Inertial"
    ros::init(argc, argv, "Stereo_Inertial");
    ros::NodeHandle n("~");  // 创建 NodeHandle，"~" 表示节点名称前缀
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);  // 设置日志级别为 Info

    // 标志是否进行图像均衡化处理
    bool bEqual = false;

    // 检查参数数量，参数少于 4 或多于 5 时给出使用说明
    if(argc < 4 || argc > 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
        ros::shutdown();
        return 1;
    }

    // 读取第三个参数，判断是否进行图像校正
    std::string sbRect(argv[3]);

    // 如果有第五个参数，检查是否进行图像均衡化处理
    if(argc == 5)
    {
        std::string sbEqual(argv[4]);
        if(sbEqual == "true")
            bEqual = true;
    }

    // 创建 ORB-SLAM3 系统实例，使用 IMU_STEREO 模式，并启用图像处理线程
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);

    // 创建 IMU 数据抓取器实例
    ImuGrabber imugb;

    // 创建图像抓取器实例，并传入 SLAM 系统、IMU 抓取器以及是否进行校正和均衡化的标志
    ImageGrabber igb(&SLAM, &imugb, sbRect == "true", bEqual);

    // 如果需要图像校正，则加载相关的校准参数
    if(igb.do_rectify)
    {
        // 从设置文件中加载立体校正相关的参数
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        // 定义左、右相机的内参矩阵、投影矩阵、旋转矩阵和畸变参数
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        // 获取图像的尺寸信息
        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        // 检查是否所有校准参数均已加载
        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        // 使用 OpenCV 的函数生成用于去畸变和立体校正的映射矩阵
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
    }

    // 订阅 IMU 数据话题，接收到的数据将传递给 ImuGrabber 的 GrabImu 函数
    ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);

    // 订阅左相机的图像数据话题
    ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);

    // 订阅右相机的图像数据话题
    ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

    // 创建一个线程，用于图像和 IMU 数据的同步处理
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    // 开始 ROS 消息循环，等待回调函数处理传入的数据
    ros::spin();

    return 0;
}

// 抓取左侧图像并存储到缓冲区
void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexLeft.lock();  // 加锁，确保线程安全

  // 如果左图像缓冲区不为空，移除最旧的图像
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();

  // 将新接收到的左图像消息推入缓冲区
  imgLeftBuf.push(img_msg);

  mBufMutexLeft.unlock();  // 解锁
}

// 抓取右侧图像并存储到缓冲区
void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRight.lock();  // 加锁，确保线程安全

  // 如果右图像缓冲区不为空，移除最旧的图像
  if (!imgRightBuf.empty())
    imgRightBuf.pop();

  // 将新接收到的右图像消息推入缓冲区
  imgRightBuf.push(img_msg);

  mBufMutexRight.unlock();  // 解锁
}

// 将 ROS 图像消息转换为 cv::Mat 格式
cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // 创建一个 cv_bridge::CvImageConstPtr 用于存储转换后的图像
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // 将 ROS 图像消息转换为 OpenCV 格式（MONO8 表示单通道 8 位图像）
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)  // 捕获转换过程中的异常
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());  // 输出错误信息
  }

  // 检查转换后的图像类型是否为 0（表示单通道灰度图像）
  if(cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();  // 返回图像的深拷贝
  }
  else
  {
    std::cout << "Error type" << std::endl;  // 输出错误类型信息
    return cv_ptr->image.clone();  // 返回图像的深拷贝
  }
}
/**
 * @brief 同步立体图像帧与IMU数据
 *
 * 此函数持续检查来自左侧和右侧相机的新图像数据以及IMU测量数据。
 * 它确保图像和IMU数据在指定的最大时间差内是同步的。
 * 如果图像和IMU数据的时间接近，则将它们一起处理以进行SLAM跟踪。
 *
 * @input 无（使用成员变量来访问图像缓冲区和IMU缓冲区）
 * @output 无（处理数据并调用SLAM系统的TrackStereo函数）
 */
void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;  // 设置最大时间差为 0.01 秒
    while(1)  // 无限循环，持续同步图像和IMU数据
    {
        cv::Mat imLeft, imRight;  // 定义左侧和右侧图像
        double tImLeft = 0, tImRight = 0;  // 初始化图像时间戳

        // 检查左图像、右图像和IMU缓冲区是否为空
        if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            // 获取左侧和右侧图像的时间戳
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            // 锁住右图像缓冲区，确保线程安全
            this->mBufMutexRight.lock();
            // 如果左图像时间戳大于右图像时间戳，且右图像缓冲区大于 1，弹出旧的右图像
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
            {
                imgRightBuf.pop();  // 移除最旧的右图像
                tImRight = imgRightBuf.front()->header.stamp.toSec();  // 更新右图像时间戳
            }
            this->mBufMutexRight.unlock();  // 解锁右图像缓冲区

            // 锁住左图像缓冲区，确保线程安全
            this->mBufMutexLeft.lock();
            // 如果右图像时间戳大于左图像时间戳，且左图像缓冲区大于 1，弹出旧的左图像
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
            {
                imgLeftBuf.pop();  // 移除最旧的左图像
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();  // 更新左图像时间戳
            }
            this->mBufMutexLeft.unlock();  // 解锁左图像缓冲区

            // 如果时间差大于最大时间差，继续下一个循环
            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;  // 可以用于调试
                continue;  // 跳过当前循环
            }

            // 如果左图像时间戳大于最后一个IMU数据时间戳，继续下一个循环
            if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            // 锁住左图像缓冲区，获取当前左图像
            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());  // 获取左图像
            imgLeftBuf.pop();  // 移除左图像
            this->mBufMutexLeft.unlock();  // 解锁左图像缓冲区

            // 锁住右图像缓冲区，获取当前右图像
            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());  // 获取右图像
            imgRightBuf.pop();  // 移除右图像
            this->mBufMutexRight.unlock();  // 解锁右图像缓冲区

            vector<ORB_SLAM3::IMU::Point> vImuMeas;  // 存储IMU测量数据
            mpImuGb->mBufMutex.lock();  // 锁住IMU缓冲区
            if (!mpImuGb->imuBuf.empty())
            {
                // 从缓冲区加载IMU测量数据
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    mpImuGb->imuBuf.front()->linear_acceleration.z);  // 加速度
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z);  // 角速度
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));  // 添加IMU测量点
                    mpImuGb->imuBuf.pop();  // 移除最旧的IMU数据
                }
            }
            mpImuGb->mBufMutex.unlock();  // 解锁IMU缓冲区

            // 如果需要直方图均衡化，应用CLAHE算法
            if (mbClahe)
            {
                mClahe->apply(imLeft, imLeft);
                mClahe->apply(imRight, imRight);
            }

            // 如果需要进行图像校正，应用重映射
            if (do_rectify)
            {
                cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
            }

            // 调用SLAM系统进行立体跟踪
            mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            std::chrono::milliseconds tSleep(1);  // 设置休眠时间为1毫秒
            std::this_thread::sleep_for(tSleep);  // 休眠
        }
    }
}

/**
 * @brief 存储IMU数据
 *
 * 此函数接收来自IMU的消息并将其存储在缓冲区中。
 * 通过互斥锁确保在存储IMU数据时的线程安全。
 *
 * @param imu_msg 输入的IMU消息，包含加速度和角速度等信息。
 * @output 无（IMU消息被存储在内部缓冲区中）
 */
void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();  // 锁住缓冲区以确保线程安全
    imuBuf.push(imu_msg);  // 将IMU消息推入缓冲区
    mBufMutex.unlock();  // 解锁缓冲区
    return;  // 返回
}


