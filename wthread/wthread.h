/*
 * @Author: BRMBC
 * @Date: 2024-03-11 19:17:16
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:45:54
 * @FilePath: /RMvision/wthread/wthread.h
 * @Description: 线程管理
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */

#ifndef WTHREAD_H
#define WTHREAD_H

#include<iostream>
#include<thread>
#include<condition_variable>
#include<mutex>

#include"mindcamera.h"
#include"serialport.h"
#include"armorDetector.h"
#include"armorBox.h"
#include"tools.h"

#define FRAME_BUFFER_NUM 6
/**
 * @brief 相机设置线程
 * @return 
 */
void* camerathread(void*);
/**
 * @brief 图像1获取线程 
 * @return
 */
void* imgthread(void*);
/**
 * @brief 图像2获取线程
 * @return  
 */
void* imgthread(void*);
/**
 * @brief 串口接收线程
 * @return 
 */
void* serialRXthread(void*);
//串口发送线程
void* serialTXthread(void*);
//装甲板处理线程
void* armorthread(void*);

void* cameradc(void*);

class Wthread
{
private:
    MindCamera camera;
    ArmorDetector detector;
//    Solver lsolve,rsolve;
    std::mutex lcamera_mutex;
    std::mutex rcamera_mutex;
    std::mutex send_mutex;
    std::queue<cv::Mat> limage_buffer;               //共享图像池
    std::queue<cv::Mat> rimage_buffer;               //共享图像池
    cv::Mat w;
    cv::Mat y;
    DebugParam debug;
//    double t1 ,t2;                                 //统计时间计算@ext:OBKoro1.korofileheader帧率
    float send_ypd[6] = {0, 0, 0, 0, 0, 0};;         //两个相机的yaw pitch distance
    char state[2] = {0, 0};
    char fire[2] = {0, 0};
    float lpitch = 0;
    float rpitch = 0;
    int num[2] = {0, 0};
    int Ltime = 0;
    int Rtime = 0;

public:
    void CameraRun();
    void Limgget(cv::Mat &l);
    void Rimgget(cv::Mat &r);
    void Larmordetector();
    void Rarmordetector();
    void LNormalConsumer(Armor &armor);
    void RNormalConsumer(Armor &armor);
    void serialRX();
    void serialTX();
    bool videosave(cv::Mat &l, cv::Mat &r);
    void img_rio(cv::Mat &l, cv::Mat &r);

};


#endif // WTHREAD_H
