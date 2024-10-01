/**
 * @file ViewerAR.h
 * @brief ORB-SLAM3 的 AR 视图处理模块
 *
 * 此文件包含 AR 视图处理的主要类和函数，处理从 SLAM 系统接收到的数据并进行可视化。
 *
 * @copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * @copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 是自由软件：您可以在 GNU 通用公共许可证第 3 版的条款下重新分发和/或修改它。
 */

#ifndef VIEWERAR_H
#define VIEWERAR_H

#include <mutex>  // 引入互斥锁以处理线程安全
#include <opencv2/core/core.hpp>  // 引入 OpenCV 核心功能
#include <pangolin/pangolin.h>  // 引入 Pangolin 库用于可视化
#include <string>
#include "../../../include/System.h"  // 引入 ORB-SLAM3 系统头文件

namespace ORB_SLAM3
{

/**
 * @class Plane
 * @brief 表示一个平面，由多个地图点定义
 */
class Plane
{
public:
    // 构造函数：通过地图点和相机位姿初始化平面
    Plane(const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcw);

    // 构造函数：通过法向量和位置初始化平面
    Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);

    // 重新计算平面的参数
    void Recompute();

    // 平面的法向量
    cv::Mat n;
    // 平面中心点
    cv::Mat o;
    // 沿法线的任意方向
    float rang;
    // 从世界坐标系到平面的变换矩阵
    cv::Mat Tpw;
    // OpenGL 矩阵，用于可视化
    pangolin::OpenGlMatrix glTpw;
    // 定义平面的地图点
    std::vector<MapPoint*> mvMPs;
    // 第一次观察平面时的相机位姿（用于计算法向量方向）
    cv::Mat mTcw, XC;
};

/**
 * @class ViewerAR
 * @brief AR 视图的主要处理类
 */
class ViewerAR
{
public:
    // 构造函数
    ViewerAR();

    // 设置帧率
    void SetFPS(const float fps){
        mFPS = fps;
        mT = 1e3 / fps;  // 计算每帧的时间间隔（毫秒）
    }

    // 设置 SLAM 系统指针
    void SetSLAM(ORB_SLAM3::System* pSystem){
        mpSystem = pSystem;
    }

    // 主线程函数，处理可视化
    void Run();

    // 设置相机内参
    void SetCameraCalibration(const float &fx_, const float &fy_, const float &cx_, const float &cy_){
        fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    }

    // 设置图像和相机位姿
    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
                      const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs);

    // 获取当前图像和相机位姿
    void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                      std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs);

private:
    // SLAM 系统指针
    ORB_SLAM3::System* mpSystem;

    // 打印状态信息
    void PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im);

    // 在图像上添加文本
    void AddTextToImage(const std::string &s, cv::Mat &im, const int r=0, const int g=0, const int b=0);

    // 加载相机位姿
    void LoadCameraPose(const cv::Mat &Tcw);

    // 绘制图像纹理
    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im);

    // 绘制立方体
    void DrawCube(const float &size, const float x=0, const float y=0, const float z=0);

    // 绘制平面
    void DrawPlane(int ndivs, float ndivsize);

    // 绘制指定的平面
    void DrawPlane(Plane* pPlane, int ndivs, float ndivsize);

    // 绘制跟踪到的关键点
    void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs, cv::Mat &im);

    // 检测平面
    Plane* DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations=50);

    // 帧率和时间
    float mFPS, mT;

    // 相机内参
    float fx, fy, cx, cy;

    // 上一帧处理的图像和相机位姿
    std::mutex mMutexPoseImage;  // 互斥锁，保证线程安全
    cv::Mat mTcw;  // 上一帧的相机位姿
    cv::Mat mImage;  // 上一帧的图像
    int mStatus;  // 状态
    std::vector<cv::KeyPoint> mvKeys;  // 关键点
    std::vector<MapPoint*> mvMPs;  // 地图点
};

}

#endif // VIEWERAR_H
