#include"mindcamera.h"
#include"CameraApi.h"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

extern int R, G, B;

//char* videopath = "//home//brmbc//视频";


/* 以下所有函数中CameraSdkStatus定义的为判断API调用返回的代码 */
bool MindCamera::MindCameraSDK()
{
    CameraSdkStatus status;
    //初始化SDK
    status = CameraSdkInit(1);
    //检测初始化是否成功
    if(status != CAMERA_STATUS_SUCCESS)
    {
        cout<<"相机SDK库初始化失败"<<endl;
        return false;
    }
    return true;
}
bool MindCamera::CameraStart()
{
    CameraSdkStatus    status;
    CameraSdkStatus    astatus;
    CameraSdkStatus    bstatus;
    //枚举设备并建立设备列表
    status = CameraEnumerateDevice(CameraList, &mDeviceCounts);
    //没有连接设备
    if(status!=CAMERA_STATUS_SUCCESS)
    {
        cout<<"没有发现设备"<<endl;
        return false;
    }
    else
    {
        cout<<"设备连接成功"<<endl;
        //相机初始化，成功后才能调用任何其他相机相关的接口
        astatus = CameraInit(&CameraList[0], -1, -1, &Camera[0]);
        bstatus = CameraInit(&CameraList[1], -1, -1, &Camera[1]);
        if(astatus != CAMERA_STATUS_SUCCESS && bstatus != CAMERA_STATUS_SUCCESS)
        {
            cout<<"相机a初始化失败，错误代码为"<<astatus<<" 相机b初始化失败，错误代码为"<<bstatus<<endl;
            return false;
        }
        else if(astatus != CAMERA_STATUS_SUCCESS && bstatus == CAMERA_STATUS_SUCCESS)
        {
            cout<<"相机a初始化失败，错误代码为"<<astatus<<" 相机b初始化成功"<<endl;
            return false;
        }
        else if(astatus == CAMERA_STATUS_SUCCESS && bstatus != CAMERA_STATUS_SUCCESS)
        {
            cout<<"相机a初始化成功"<<" 相机b初始化失败，错误代码为"<<bstatus<<endl;
            return false;
        }
        else
        {
            cout<<"相机a和相机b初始化成功"<<endl;
            return true;
        }
    }
}

bool MindCamera::CameraInfoGet()
{
    //获得相机的特性描述结构体,该结构体中包含了相机可设置的各种参数的范围信息,决定了相关函数的参数
    CameraGetCapability(Camera[0], &a_Capability);
    CameraGetCapability(Camera[1], &b_Capability);
    //获取图像数据大小
    aRgbBuffer = (unsigned char*)malloc(a_Capability.sResolutionRange.iHeightMax*a_Capability.sResolutionRange.iWidthMax
                                        *3);
    bRgbBuffer = (unsigned char*)malloc(b_Capability.sResolutionRange.iHeightMax*b_Capability.sResolutionRange.iWidthMax
                                        *3);

    //设置图像格式
    CameraSetIspOutFormat(Camera[0], CAMERA_MEDIA_TYPE_BGR8);
    CameraSetIspOutFormat(Camera[1], CAMERA_MEDIA_TYPE_BGR8);
    return true;
}

bool MindCamera::CameraRun()
{
    CameraSdkStatus astatus;
    CameraSdkStatus bstatus;
    //让SDK内部取图线程开始工作
    astatus = CameraPlay(Camera[0]);
    bstatus = CameraPlay(Camera[1]);
    if(astatus != CAMERA_STATUS_SUCCESS && bstatus != CAMERA_STATUS_SUCCESS)
    {
        cout<<"相机图像开采失败,失败代码为"<<astatus<<"和"<<bstatus<<endl;
        //关闭相机
        CameraStop(Camera[0]);
        CameraStop(Camera[1]);
        CameraUnInit(Camera[0]);
        CameraUnInit(Camera[1]);
        return false;
    }
    else
    {
        cout<<"图像开采成功"<<endl;
        return true;
    }
}

bool MindCamera::lCameraGet(Mat &w)
{
    CameraSdkStatus astatus;
    astatus = CameraGetImageBuffer(Camera[0], &a_FrameHead, &aRawBuffer, 300);
    if(astatus == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(Camera[0], aRawBuffer, aRgbBuffer, &a_FrameHead);
        Mat a(Size(a_FrameHead.iWidth,a_FrameHead.iHeight), CV_8UC3, aRgbBuffer);
        a.copyTo(w);
        CameraReleaseImageBuffer(Camera[0], aRawBuffer);
        return true;
    }
    else
    {
        cout<<"左读取一帧图像失败,错误代码为"<<astatus<<endl;
        CameraReleaseImageBuffer(Camera[0], aRawBuffer);
        CameraUnInit(Camera[0]);
        return false;
    }
    return true;
}

bool MindCamera::rCameraGet(Mat &y)
{
    CameraSdkStatus bstatus;
    bstatus = CameraGetImageBuffer(Camera[1], &b_FrameHead, &bRawBuffer, 300);
    if(bstatus == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(Camera[1], bRawBuffer, bRgbBuffer, &b_FrameHead);
        Mat b(Size(b_FrameHead.iWidth, b_FrameHead.iHeight), CV_8UC3, bRgbBuffer);
        b.copyTo(y);
        CameraReleaseImageBuffer(Camera[1], bRawBuffer);
        return true;
    }
    else
    {
        cout<<"右读取一帧图像失败,错误代码为"<<bstatus<<endl;
        CameraReleaseImageBuffer(Camera[1], bRawBuffer);
        CameraUnInit(Camera[1]);
        return false;
    }
    return true;
}

bool MindCamera::CameraSetResolution(int offsetx,int offsety,int width,int height)
{
    CameraSdkStatus astatus;
    CameraSdkStatus bstatus;
    //设置0xff自定义分辨率
    mResolution.iIndex = 0xff;
    //相机实际输出宽度
    mResolution.iWidth = width;
    //相机实际输出高度
    mResolution.iHeight = height;
    //视场实际宽度
    mResolution.iWidthFOV = width;
    //视场实际高度
    mResolution.iHeightFOV = height;
    //视场宽度偏移
    mResolution.iHOffsetFOV = offsetx;
    //视场高度偏移
    mResolution.iVOffsetFOV = offsety;
    //ISP软件缩放宽高，都为0表示不缩放
    mResolution.iWidthZoomSw = 0;
    mResolution.iHeightZoomSw = 0;
    //BIN SKIP 模式设置(需要相机支持)
    mResolution.uBinAverageMode = 0;
    mResolution.uBinSumMode = 0;
    mResolution.uResampleMask = 0;
    mResolution.uSkipMode = 0;

    astatus = CameraSetImageResolution(Camera[0],&mResolution);
    bstatus = CameraSetImageResolution(Camera[1],&mResolution);
    if(astatus == CAMERA_STATUS_SUCCESS && bstatus == CAMERA_STATUS_SUCCESS)
    {
        cout<<"分辨率设置成功"<<endl;
        return true;
    }
    else
    {
        cout<<"分辨率设置失败，错误代码为"<<astatus<<" "<<bstatus<<endl;
        CameraUnInit(Camera[0]);
        CameraUnInit(Camera[1]);
        return false;
    }
}

/*  ExposureTime为曝光时间，单位为us  */
bool MindCamera::CameraSetExposuretime(double ExposureTime)
{
    CameraSdkStatus astatus;
    CameraSdkStatus bstatus;
//    astatus = CameraSetAeState(aCamera,TRUE);
//    bstatus = CameraSetAeState(bCamera,TRUE);
    astatus = CameraSetAeState(Camera[0],FALSE);
    bstatus = CameraSetAeState(Camera[1],FALSE);
    if(astatus == CAMERA_STATUS_SUCCESS && bstatus == CAMERA_STATUS_SUCCESS)
    {
        CameraSetExposureTime(Camera[0],ExposureTime);
        CameraSetExposureTime(Camera[1],ExposureTime);
        cout<<"曝光时间设置成功"<<endl;
//        CameraGetExposureLineTime(mCamera,&m_ExpLineTime);
//        cout<<m_ExpLineTime<<endl;
        return true;
    }
    else
    {
        cout<<"自动曝光设置失败，错误代码为"<<astatus<<"和"<<bstatus<<endl;
        CameraUnInit(Camera[0]);
        CameraUnInit(Camera[1]);
        return false;
    }
}

/*  实际的放大倍数是设定值 Exp / 100  */
bool MindCamera::CameraSetGAIN(int ExpR, int ExpG, int ExpB)
{
    CameraSdkStatus astatus;
    CameraSdkStatus bstatus;

    int aR = 0, aG = 0, aB = 0, bR = 0, bG = 0, bB = 0;

    astatus = CameraSetGain(Camera[0], ExpR, ExpG, ExpB);
    bstatus = CameraSetGain(Camera[1], ExpR, ExpG, ExpB);

    if(astatus == CAMERA_STATUS_SUCCESS && bstatus == CAMERA_STATUS_SUCCESS)
    {
        CameraGetGain(Camera[0],&aR,&aG,&aB);
        CameraGetGain(Camera[1],&bR,&bG,&bB);
        //cout<<"相机a增益为"<<aR<<","<<aG<<","<<aB<<"\n"<<"相机b增益为"<<bR<<","<<bG<<","<<bB<<endl;
        if(aG > 200 || bG > 200)
        {
            CameraSetGain(Camera[0], ExpR, ExpG, ExpB);
            CameraSetGain(Camera[1], ExpR, ExpG, ExpB);
        }
        cout<<"增益设置成功"<<endl;
        return true;
    }
    else
        return false;
    return true;
}

bool MindCamera::CameraSetBALANCE_MODE(int m)
{
    CameraSdkStatus astatus;
    CameraSdkStatus bstatus;
    int a;
    int b;
    CameraGetWbMode(Camera[0],&a);
    CameraGetWbMode(Camera[1],&b);
    cout<<a<<endl;
    cout<<b<<endl;
    astatus = CameraSetWbMode(Camera[0],m);
    bstatus = CameraSetWbMode(Camera[1],m);
    if(astatus == 0 && bstatus == 0)
    {
        switch (m)
        {
        case 0:
            cout<<"手动白平衡设置成功"<<endl;
            break;
        case 1:
            cout<<"自动白平衡设置成功"<<endl;
            break;
        }
        return true;
    }
    else
    {
        cout<<"白平衡设置失败，错误代码为"<<astatus<<"和"<<bstatus<<endl;
        CameraUnInit(Camera[0]);
        CameraUnInit(Camera[1]);
        return false;
    }
}

bool MindCamera::CameraSetCTMode(int m)
{
    CameraSdkStatus astatus ;
    CameraSdkStatus bstatus ;
    //需要不同模式参考emSdkClrTmpMode(CameraDefine.h  234行)
    astatus = CameraSetClrTempMode(Camera[0],m);
    bstatus = CameraSetClrTempMode(Camera[1],m);
    if(astatus == 0 && bstatus == 0)
    {
        switch (m)
        {
        case 0:
            cout<<"白平衡色温模式为自动模式"<<endl;
            break;
        case 1:
            cout<<"白平衡色温模式为预设模式"<<endl;
            break;
        case 2:
            cout<<"白平衡色温模式模式自定义模式"<<endl;
            break;
        }
        return true;
    }
    else
    {
        cout<<"白平衡色温模式错误，错误代码为"<<astatus<<endl;
        return false;
    }
}

bool MindCamera::CameraSetFPS(bool f)
{
    if(f)
    {
        CameraGetFrameStatistic(Camera[0],&a_FPS);
        CameraGetFrameStatistic(Camera[1],&b_FPS);
        a_display_fps = a_FPS.iCapture;
        b_display_fps = b_FPS.iCapture;
//        cout<<"相机a帧率为"<<a_read_fps<<"\n"<<"相机b帧率为"<<b_read_fps<<endl;
        cout<<"相机a帧率为"<<a_display_fps<<"\n"<<"相机b帧率为"<<b_display_fps<<endl;
        return true;
    }
    else
    {
        return false;
    }
}

// bool MindCamera::CameraVideo()
// {
//     CameraSdkStatus astatus;

//     astatus = CameraInitRecord(Camera[0], 0, videopath, TRUE, 90, 300);

//     if(astatus == CAMERA_STATUS_SUCCESS)
//     {
//         cout<<"视频录制成功"<<endl;
//         return true;
//     }
//     else
//     {
//         cout<<"视频录制失败，错误代码为"<<astatus<<endl;

//         return false;
//     }
// }

bool MindCamera::UintMindCamera()
{
    CameraSdkStatus astatus;
    CameraSdkStatus bstatus;
    //停止开采图像
    astatus = CameraStop(Camera[0]);
    bstatus = CameraStop(Camera[1]);
    //释放相机
    astatus = CameraUnInit(Camera[0]);
    bstatus = CameraUnInit(Camera[1]);
    aRawBuffer = NULL;
    bRawBuffer = NULL;
    if(astatus == CAMERA_STATUS_SUCCESS && bstatus == CAMERA_STATUS_SUCCESS)
    {
        free(aRawBuffer);
        free(bRawBuffer);
        cout<<"相机资源释放成功"<<endl;
        return true;
    }
    else
        return false;
}
bool MindCamera::Run()
{
    MindCameraSDK();
    if(!CameraStart())
        return false;
    this_thread::sleep_for(chrono::milliseconds(10));
    if(!CameraInfoGet())
        return false;
    this_thread::sleep_for(chrono::milliseconds(10));
    if(!CameraSetResolution(0,0,1280,1024))
        return false;
    this_thread::sleep_for(chrono::milliseconds(10));
    if(!CameraSetExposuretime(e))
        return false;
    this_thread::sleep_for(chrono::milliseconds(10));
//    if(!CameraSetCTMode(1))
//        return false;
    //CameraSetBALANCE_MODE(1);  //有问题
     //所有通道
    if(!CameraSetGAIN(R, G, B))
        return false;
    this_thread::sleep_for(chrono::milliseconds(10));
     //单独通道
//     if(!CameraSetGAIN(0, 160, 120, 100))
//         return false;
//     if(!CameraSetGAIN(1, 160, 120, 100))
//         return false;
//     if(!CameraSetGAIN(2, 160, 120, 100))
//         return false;
    // if(!CameraVideo())
    //     return false;
    if(!CameraRun())
        return false;
    return true;
}
