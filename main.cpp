/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:01:44
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:45:52
 * @FilePath: /RMvision/main.cpp
 * @Description: 主函数
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */

#include<iostream>
#include<pthread.h>
#include<thread>

#include"wthread/wthread.h"
#include"Serial/serialport.h"
#include"extools/tools.h"


Color ENEMYCOLOR = BLUE;                                   //设置敌方颜色
int targetNum;                                            //目标数字
double e = 4*1000;                                        //手动设置曝光值(单位us)
int R = 140, G = 120, B = 100;                            //RGB颜色通道增益

using namespace cv;
using namespace std;

int main()
{
    Wthread main_progress;

    thread imageget(&Wthread::CameraRun, &main_progress);
    thread larmor(&Wthread::Larmordetector, &main_progress);
    thread rarmor(&Wthread::Rarmordetector,&main_progress);
    thread read(&Wthread::serialRX, &main_progress);
    thread send(&Wthread::serialTX, &main_progress);

    imageget.join();
    larmor.join();
    rarmor.join();
    read.join();
    send.join();


    return 1;
}
