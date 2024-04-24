/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:00:46
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-22 04:41:50
 * @FilePath: /RMvision/camera_device/include/mindcamera.h
 * @Description: 
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#ifndef MINDCAMERA_H
#define MINDCAMERA_H

#include<iostream>
#include<opencv2/core/core.hpp>

#include"CameraApi.h"

extern double e;

class MindCamera
{
private:

    INT                     mDeviceCounts = 2;         //相机设备数量 >> 用几个填几个
    tSdkCameraDevInfo       CameraList[2];             //设备详细信息 >> []有几个设备填几个
    int                     Camera[2];                 //相机句柄    >> []有几个设备填几个
    BYTE                    * aRawBuffer;              //设备a处理前图像数据缓冲区
    unsigned char           * aRgbBuffer;              //设备a处理后图像数据缓冲区
    BYTE                    * bRawBuffer;              //设备b处理前图像数据缓冲区
    unsigned char           * bRgbBuffer;              //设备b处理后图像数据缓冲区
    tSdkFrameHead           a_FrameHead;               //设备a图像帧头信息
    tSdkFrameHead           b_FrameHead;               //设备b图像帧头信息
    tSdkCameraCapbility     a_Capability;              //设备a描述信息
    tSdkCameraCapbility     b_Capability;              //设备b描述信息
    tSdkImageResolution     mResolution;               //图像分辨率初始化设置信息
    //未用上数据
    tSdkFrameStatistic      a_FPS;                     //相机a帧率信息
    tSdkFrameStatistic      b_FPS;                     //相机b帧率信息
    int                     a_read_fps=0;              //相机a统计读取帧率
    int                     a_display_fps=0;           //相机a统计显示帧率
    int                     b_read_fps=0;              //相机b统计读取帧率
    int                     b_display_fps=0;           //相机b统计显示帧率

private:
    /**
     * 函数名称: MindCameraSDK()
     * 功能描述: 相机SDK初始化
     * 参数说明: 无
     * 返回值: 无
     * 额外说明: 初始化失败回报错并终止程序
    */
    bool MindCameraSDK();
    /**
     * 函数名称: CameraStart()
     * 功能描述: 枚举设备并连接设备
     * 参数说明: 无
     * 返回值: 无
     * 额外说明: 未识别设备或连接设备失败会报错并终止程序
    */
    bool CameraStart();
    /**
     * 函数名称: CameraGet()
     * 功能描述: 获取相机设置信息然后打开摄像头设置图像格式
     * 参数说明:  ->
     * 返回值:
     * 额外说明:
    */
    bool CameraInfoGet();
    //开始采集
    bool CameraRun();
    //设置分辨率，支持最大1280*1024,最小320*240；默认1280*1024
    bool CameraSetResolution(int offsetx,int offsety,int width=1280,int heitht=1024);
    //设置手动曝光值
    bool CameraSetExposuretime(double ExposureTime);
    //设置手动曝光增益
    bool CameraSetGAIN(int ExpR, int ExpG, int ExpB);
    //设置白平衡模式，1是自动白平衡，0是手动白平衡，默认0(有问题) >> SDK返回报错显示不支持该功能 >> 弃用
    bool CameraSetBALANCE_MODE(int m = 0);
    //设置色温模式，
    bool CameraSetCTMode(int m);
    //获取相机帧率
    bool CameraSetFPS(bool f);

public:
    //获取左相机每帧图像
    bool lCameraGet(cv::Mat &w);
    //获取右相机每帧图像
    bool rCameraGet(cv::Mat &y);
    //
    bool CameraVideo();
    //
    bool Run();
    //关闭相机释放资源
    bool UintMindCamera();
    //
    MindCamera() = default;
};

#endif // MINDCAMERA_H
