/**
* 该文件是 ORB-SLAM3 项目的一部分
*
* ORB-SLAM3 是自由软件：你可以根据 GNU 通用公共许可证的条款重新发布和/或修改它。
* 本程序按"现状"提供，没有任何保证。有关详细信息，请参见 GNU 通用公共许可证。
*/

#include "ViewerAR.h" // 引入 AR 查看器相关的头文件
#include <opencv2/highgui/highgui.hpp> // OpenCV 显示图片的头文件
#include <Eigen/Dense> // 使用 Eigen 库进行矩阵运算
#include <opencv2/core/eigen.hpp> // OpenCV 与 Eigen 库的相互转换
#include <mutex> // 多线程同步相关
#include <thread> // 多线程相关
#include <cstdlib> // 标准库

using namespace std;

namespace ORB_SLAM3
{

const float eps = 1e-4; // 用于避免数值计算不稳定的小值

// 计算 SO3（特殊正交群）的指数映射，输入为旋转向量的三个分量 x, y, z
cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F); // 生成一个 3x3 单位矩阵
    const float d2 = x*x + y*y + z*z; // 旋转向量的模平方
    const float d = sqrt(d2); // 旋转向量的模（长度）

    // 构造反对称矩阵 W
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                                         z, 0, -x,
                                        -y,  x, 0);
    // 当旋转向量非常小时，采用近似公式
    if(d < eps)
        return (I + W + 0.5f * W * W); // 泰勒展开的二次近似
    else
        return (I + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2); // 标准 Rodrigues 公式
}

// 重载函数，输入为 3x1 的旋转向量
cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0), v.at<float>(1), v.at<float>(2)); // 从矩阵中提取旋转向量的分量
}

// ViewerAR 类的构造函数，进行必要的初始化
ViewerAR::ViewerAR(){}

// ViewerAR 的主循环，负责显示与交互
void ViewerAR::Run()
{
    int w, h, wui; // 定义图像宽度、高度和 UI 面板宽度

    cv::Mat im, Tcw; // 定义图像和相机位姿矩阵
    int status; // 表示系统当前状态（如 SLAM 正常、VO 等）
    vector<cv::KeyPoint> vKeys; // 保存图像中的关键点
    vector<MapPoint*> vMPs; // 保存三维地图点

    // 等待获取不为空的图像
    while(1)
    {
        GetImagePose(im, Tcw, status, vKeys, vMPs); // 获取最新的图像、相机位姿和状态
        if(im.empty()) // 如果图像为空，等待一段时间
            cv::waitKey(mT);
        else
        {
            w = im.cols; // 获取图像宽度
            h = im.rows; // 获取图像高度
            break; // 退出循环
        }
    }

    wui = 200; // UI 面板的宽度

    // 创建一个用于显示的窗口，并绑定 OpenGL 上下文
    pangolin::CreateWindowAndBind("Viewer", w + wui, h);

    // 启用深度测试和混合模式
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);

    // 创建用于 UI 面板的控件和菜单项
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(wui));
    pangolin::Var<bool> menu_detectplane("menu.Insert Cube", false, false); // 插入立方体
    pangolin::Var<bool> menu_clear("menu.Clear All", false, false); // 清除所有立方体
    pangolin::Var<bool> menu_drawim("menu.Draw Image", true, true); // 是否绘制图像
    pangolin::Var<bool> menu_drawcube("menu.Draw Cube", true, true); // 是否绘制立方体
    pangolin::Var<float> menu_cubesize("menu.Cube Size", 0.05, 0.01, 0.3); // 立方体尺寸控制
    pangolin::Var<bool> menu_drawgrid("menu.Draw Grid", true, true); // 是否绘制网格
    pangolin::Var<int> menu_ngrid("menu.Grid Elements", 3, 1, 10); // 网格元素数量
    pangolin::Var<float> menu_sizegrid("menu.Element Size", 0.05, 0.01, 0.3); // 网格元素大小
    pangolin::Var<bool> menu_drawpoints("menu.Draw Points", false, true); // 是否绘制跟踪点

    // 本地化模式开关
    pangolin::Var<bool> menu_LocalizationMode("menu.Localization Mode", false, true);
    bool bLocalizationMode = false; // 初始化本地化模式标志

    // 设置用于显示图像的 OpenGL 视口
    pangolin::View& d_image = pangolin::Display("image")
        .SetBounds(0, 1.0f, pangolin::Attach::Pix(wui), 1.0f, (float)w/h)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::GlTexture imageTexture(w, h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE); // 创建 OpenGL 纹理对象，用于显示图像

    // 设置相机投影矩阵
    pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(w, h, fx, fy, cx, cy, 0.001, 1000);

    vector<Plane*> vpPlane; // 保存虚拟物体的平面

    // 主循环
    while(1)
    {
        // 检查并更新本地化模式状态
        if(menu_LocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode(); // 激活本地化模式
            bLocalizationMode = true;
        }
        else if(!menu_LocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode(); // 取消本地化模式
            bLocalizationMode = false;
        }

        // 清除颜色和深度缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 激活图像显示区域
        d_image.Activate();
        glColor3f(1.0, 1.0, 1.0); // 设置绘制颜色为白色

        // 获取最新的图像和相机位姿
        GetImagePose(im, Tcw, status, vKeys, vMPs);

        // 在图像上显示状态信息
        PrintStatus(status, bLocalizationMode, im);

        // 如果开启了绘制跟踪点选项，绘制跟踪点
        if(menu_drawpoints)
            DrawTrackedPoints(vKeys, vMPs, im);

        // 如果开启了绘制图像选项，绘制图像
        if(menu_drawim)
            DrawImageTexture(imageTexture, im);

        // 清除深度缓冲区
        glClear(GL_DEPTH_BUFFER_BIT);

        // 加载相机投影矩阵
        glMatrixMode(GL_PROJECTION);
        P.Load();

        // 切换到模型视图矩阵
        glMatrixMode(GL_MODELVIEW);

        // 加载相机位姿矩阵
        LoadCameraPose(Tcw);

        // 绘制虚拟物体（立方体、网格等）
        if(status == 2)
        {
            // 如果选择了清除立方体，删除所有虚拟平面
            if(menu_clear)
            {
                if(!vpPlane.empty())
                {
                    for(size_t i = 0; i < vpPlane.size(); i++)
                    {
                        delete vpPlane[i]; // 删除每个平面
                    }
                    vpPlane.clear(); // 清空平面列表
                    cout << "All cubes erased!" << endl;
                }
                menu_clear = false;
            }

            // 如果选择了插入立方体，检测平面并插入新的虚拟物体
            if(menu_detectplane)
            {
                Plane* pPlane = DetectPlane(Tcw, vMPs, 50); // 检测一个平面
                if(pPlane)
                {
                    cout << "New virtual cube inserted!" << endl;
                    vpPlane.push_back(pPlane); // 将平面添加到列表中
                }
                else
                {
                    cout << "No plane detected. Point the camera to a planar region." << endl;
                }
                menu_detectplane = false;
            }

            // 绘制虚拟物体
            if(!vpPlane.empty())
            {
                // 如果发生了闭环或者全局优化，需要重新计算虚拟物体的位置
                bool bRecompute = false;
                if(!bLocalizationMode)
                {
                    if(mpSystem->MapChanged())
                    {
                        cout << "Map changed. All virtual elements are recomputed!" << endl;
                        bRecompute = true;
                    }
                }

                // 遍历所有虚拟平面，绘制虚拟立方体和网格
                for(size_t i = 0; i < vpPlane.size(); i++)
                {
                    Plane* pPlane = vpPlane[i];

                    if(pPlane)
                    {
                        if(bRecompute)
                        {
                            pPlane->Recompute(); // 重新计算平面
                        }
                        glPushMatrix(); // 保存当前矩阵状态
                        pPlane->glTpw.Multiply(); // 加载平面的位姿矩阵

                        // 如果开启了绘制立方体选项，绘制立方体
                        if(menu_drawcube)
                        {
                            DrawCube(menu_cubesize);
                        }

                        // 如果开启了绘制网格选项，绘制网格
                        if(menu_drawgrid)
                        {
                            DrawPlane(menu_ngrid, menu_sizegrid);
                        }

                        glPopMatrix(); // 恢复矩阵状态
                    }
                }
            }
        }

        pangolin::FinishFrame(); // 完成当前帧绘制
        usleep(mT * 1000); // 休眠一段时间，控制帧率
    }
}


void ViewerAR::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM3::MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    mImage = im.clone();
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
    mvMPs = vMPs;
}

void ViewerAR::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    im = mImage.clone();
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;
}

void ViewerAR::LoadCameraPose(const cv::Mat &Tcw)
{
    if(!Tcw.empty())
    {
        pangolin::OpenGlMatrix M;

        M.m[0] = Tcw.at<float>(0,0);
        M.m[1] = Tcw.at<float>(1,0);
        M.m[2] = Tcw.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Tcw.at<float>(0,1);
        M.m[5] = Tcw.at<float>(1,1);
        M.m[6] = Tcw.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Tcw.at<float>(0,2);
        M.m[9] = Tcw.at<float>(1,2);
        M.m[10] = Tcw.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = Tcw.at<float>(0,3);
        M.m[13] = Tcw.at<float>(1,3);
        M.m[14] = Tcw.at<float>(2,3);
        M.m[15]  = 1.0;

        M.Load();
    }
}

void ViewerAR::PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im)
{
    if(!bLocMode)
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("SLAM ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("SLAM LOST",im,255,0,0); break;}
        }
    }
    else
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("LOCALIZATION ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("LOCALIZATION LOST",im,255,0,0); break;}
        }
    }
}

void ViewerAR::AddTextToImage(const string &s, cv::Mat &im, const int r, const int g, const int b)
{
    int l = 10;
    //imText.rowRange(im.rows-imText.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);

    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2,8);
}

void ViewerAR::DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im)
{
    if(!im.empty())
    {
        imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
        imageTexture.RenderToViewportFlipY();
    }
}

void ViewerAR::DrawCube(const float &size,const float x, const float y, const float z)
{
    pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x,-size-y,-z);
    glPushMatrix();
    M.Multiply();
    pangolin::glDrawColouredCube(-size,size);
    glPopMatrix();
}

void ViewerAR::DrawPlane(Plane *pPlane, int ndivs, float ndivsize)
{
    glPushMatrix();
    pPlane->glTpw.Multiply();
    DrawPlane(ndivs,ndivsize);
    glPopMatrix();
}

void ViewerAR::DrawPlane(int ndivs, float ndivsize)
{
    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        glVertex3f(minx+ndivsize*n,0,minz);
        glVertex3f(minx+ndivsize*n,0,maxz);
        glVertex3f(minx,0,minz+ndivsize*n);
        glVertex3f(maxx,0,minz+ndivsize*n);
    }

    glEnd();

}

void ViewerAR::DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint *> &vMPs, cv::Mat &im)
{
    const int N = vKeys.size();


    for(int i=0; i<N; i++)
    {
        if(vMPs[i])
        {
            cv::circle(im,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
        }
    }
}

Plane* ViewerAR::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations)
{
    // Retrieve 3D points
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());
    vector<MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>5)
            {
                cv::Mat WorldPos;
                cv::eigen2cv(pMP->GetWorldPos(), WorldPos);
                vPoints.push_back(WorldPos);

            }
        }
    }

    const int N = vPoints.size();

    if(N<50)
        return NULL;


    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;

    //RANSAC
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            A.row(i).colRange(0,3) = vPoints[idx].t();

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

        for(int i=0; i<N; i++)
        {
            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;
        }

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(),vSorted.end());

        int nth = max((int)(0.2*N),20);
        const float medianDist = vSorted[nth];

        if(medianDist<bestDist)
        {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    const float th = 1.4*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++)
    {
        if(bestvDist[i]<th)
        {
            nInliers++;
            vbInliers[i]=true;
        }
    }

    vector<MapPoint*> vInlierMPs(nInliers,NULL);
    int nin = 0;
    for(int i=0; i<N; i++)
    {
        if(vbInliers[i])
        {
            vInlierMPs[nin] = vPointMP[i];
            nin++;
        }
    }

    return new Plane(vInlierMPs,Tcw);
}

Plane::Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
{
    rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    Recompute();
}

void Plane::Recompute()
{
    const int N = mvMPs.size();

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvMPs[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw;
            cv::eigen2cv(pMP->GetWorldPos(), Xw);
            o+=Xw;
            A.row(nPoints).colRange(0,3) = Xw.t();
            nPoints++;
        }
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    o = o*(1.0f/nPoints);
    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    Tpw = cv::Mat::eye(4,4,CV_32F);


    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;

}

Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
{
    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);
    o = (cv::Mat_<float>(3,1)<<ox,oy,oz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float s = cv::norm(v);
    const float c = up.dot(n);
    const float a = atan2(s,c);
    Tpw = cv::Mat::eye(4,4,CV_32F);
    const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    cout << rang;
    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;
}

}
