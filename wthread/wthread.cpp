/*
 * @Author: BRMBC
 * @Date: 2024-03-11 19:17:16
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:46:07
 * @FilePath: /RMvision/wthread/wthread.cpp
 * @Description: 线程管理
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */
#include "wthread.h"
#include "serialport.h"
#include "mindcamera.h"
#include "armorDetector.h"

#include<iostream>
#include<opencv2/highgui.hpp>
#include<chrono>
#include<condition_variable>
#include<mutex>

using namespace cv;
using namespace std;

//Mat w,y;                                        //用于接受的图像
//Mat img;                                        //用于发送的图像

extern Color ENEMYCOLOR;
extern int targetNum;
extern SerialPort usart;
//extern float ldpitch, rdpitch; 

bool close = 0;

void Wthread::CameraRun()
{
    if(camera.Run())
    {
        while(1)
        {
            //double t1 = getTickCount();
            if(!camera.lCameraGet(y) || !camera.rCameraGet(w))
                break;
            if(w.empty() || y.empty())
                continue;
            img_rio(w ,y);
            Limgget(w);
            Rimgget(y);
            //double t2 = (getTickCount()-t1)/getTickFrequency()
            //printf("Armor Detecting FPS: %f\n",1/t2);
            if(waitKey(1) == 27)
            {
                destroyAllWindows();
                break;
            }
            if(debug.debug_img_show)
            {
                imshow("L", w);
                imshow("R", y);
                //videosave(w, y);
            }
            //this_thread::sleep_for(chrono::milliseconds(1));
        }
        camera.UintMindCamera();
        exit(0);
    }
    else
    {
        camera.UintMindCamera();
        exit(0);
    }
}

void Wthread::Limgget(cv::Mat &l)
{
    lcamera_mutex.lock();
    limage_buffer.push(l);
    while(limage_buffer.size() > FRAME_BUFFER_NUM)
    {
        limage_buffer.pop();
    }
    lcamera_mutex.unlock();
}

void Wthread::Rimgget(cv::Mat &r)
{
    rcamera_mutex.lock();
    rimage_buffer.push(r);
    while(rimage_buffer.size() > FRAME_BUFFER_NUM)
    {
        rimage_buffer.pop();
    }
    rcamera_mutex.unlock();
}

void Wthread::Larmordetector()
{
    this_thread::sleep_for(chrono::milliseconds(1500));
    Armor Larmor;
    while(1)
    {
        while(limage_buffer.empty())
            this_thread::sleep_for(chrono::milliseconds(1));
//        unique_lock<mutex> lock(camera_mutex);
        //double t1 = getTickCount();
        lcamera_mutex.lock();
        Larmor.img = limage_buffer.front();
        limage_buffer.pop();
        lcamera_mutex.unlock();
        try
        {
            LNormalConsumer(Larmor);
        }
        catch(cv::Exception& e)
        {
            std::cerr << "opencv error:" << e.what() << std::endl;
        }
        catch(std::exception& e)
        {
            std::cerr << "std error:" << e.what() << std::endl;
        }
        //double t2 = (getTickCount()-t1)/getTickFrequency();
        //printf("L Armor Detecting FPS: %f\n",1/t2);
    }
}

void Wthread::LNormalConsumer(Armor &armor)
{
    //double t1 = getTickCount();
    detector.LRun(armor);
    //double t2 = (getTickCount()-t1)/getTickFrequency();
    //printf("L Armor Detecting FPS: %f\n",1/t2);
    send_mutex.lock();
    state[0] = armor.found;
    num[0] = armor.id;
    fire[0] = armor.fire;
    send_ypd[0] = armor.ypd.x;
    send_ypd[1] = armor.ypd.y;
    send_ypd[2] = armor.ypd.z;
    send_mutex.unlock();
}

void Wthread::Rarmordetector()
{
    this_thread::sleep_for(chrono::milliseconds(1500));
    Armor Rarmor;
    while(1)
    {
        while(rimage_buffer.empty())
            this_thread::sleep_for(chrono::milliseconds(1));
        //unique_lock<mutex> lock(camera_mutex);
        //double t1 = getTickCount();
        rcamera_mutex.lock();
        Rarmor.img = rimage_buffer.front();
        rimage_buffer.pop();
        rcamera_mutex.unlock();
        try
        {
            RNormalConsumer(Rarmor);
        }
        catch (cv::Exception& e)
        {
            std::cerr << "opencv error:" << e.what() << std::endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "std error:" << e.what() << std::endl;
        }
        //double t2=(getTickCount()-t1)/getTickFrequency();
//         printf("R Armor Detecting FPS: %f\n",1/t2);
    }
}

void Wthread::RNormalConsumer(Armor &armor)
{
    // Rread_mutex.lock();
    // armor.pitch = rpitch_buffer.front();
    // rpitch_buffer.pop();
    // Rread_mutex.unlock();
    detector.RRun(armor);
    send_mutex.lock();
    state[1] = armor.found;
    num[1] = armor.id;
    fire[1] = armor.fire;
    send_ypd[3] = armor.ypd.x;
    send_ypd[4] = armor.ypd.y;
    send_ypd[5] = armor.ypd.z;
    send_mutex.unlock();
}

void Wthread::serialRX()
{
    this_thread::sleep_for(chrono::seconds(1));
    usart_rx_t usart_rx_temp;
    unsigned char buff[100];
    while(1)
    {
        if(usart.read_data(buff,sizeof(usart_rx_t))==sizeof(usart_rx_t))
        {
            memcpy(&usart_rx_temp, buff, sizeof(usart_rx_t));
            if(usart_rx_temp.head == 0xAB && usart_rx_temp.end == 0xBA)
            {
                if(usart_rx_temp.color == 7)
                    ENEMYCOLOR = BLUE;
                else if(usart_rx_temp.color == 107)
                    ENEMYCOLOR = RED;
                //ldpitch = usart_rx_temp.lpitch;
                //rdpitch = usart_rx_temp.rpitch;
            }
            // Lread(lpitch);
            // Rread(rpitch);
        }

        //cout<<"data:"<<to_string(usart_rx_temp.color)<<" color:"<<to_string(ENEMYCOLOR)<<endl;
        //cout<<"pitch:"<<ldpitch<<" "<<rdpitch<<endl;
        this_thread::sleep_for(chrono::microseconds(10));
    }
}

void Wthread::serialTX()
{
    this_thread::sleep_for(chrono::seconds(5));
    while(1)
    {
        if(debug.debug_mode)
        {
            send_mutex.lock();
            usart.ready_send(0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1);
            usart.send_data();
            send_mutex.unlock();
        }
        else
        {
            send_mutex.lock();
            usart.ready_send(state[0], 0, fire[0], send_ypd[0], send_ypd[1], send_ypd[2],
                             state[1], 0, fire[1], send_ypd[3], send_ypd[4], send_ypd[5]);
//测试
//            if(found[0] == 1)
//                cout<<"L:"<<send_ypd[0]<<" "<<send_ypd[1]<<" "<<send_ypd[2]<<endl;
//            if(found[1] == 1)
//                cout<<"R:"<<send_ypd[3]<<" "<<send_ypd[4]<<" "<<send_ypd[5]<<endl;
            usart.send_data();
            if(debug.debug_send_data)
            {
                //测试
                // cout<<"L:"<<found[0]<<" "<<num[0]<<" "<<send_ypd[0]<<" "<<send_ypd[1]<<" "<<send_ypd[2]<<endl;
                // cout<<"R:"<<found[1]<<" "<<num[1]<<" "<<send_ypd[3]<<" "<<send_ypd[4]<<" "<<send_ypd[5]<<endl;
                cout<<"L:"<<to_string(state[0])<<" "<<to_string(fire[0])<<" "<<send_ypd[0]<<" "<<send_ypd[1]<<" "<<send_ypd[2]<<endl;
                cout<<"R:"<<to_string(state[1])<<" "<<to_string(fire[1])<<" "<<send_ypd[3]<<" "<<send_ypd[4]<<" "<<send_ypd[5]<<endl;
            }
            send_mutex.unlock();
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

void Wthread::img_rio(cv::Mat &l, cv::Mat &r)
{
    for(int h = 0; h < 250; h++)
    {
        for(int w = 0; w < 1280; w++)
        {
            l.at<cv::Vec3b>(h,w) = cv::Vec3b(0,0,0);
            r.at<cv::Vec3b>(h,w) = cv::Vec3b(0,0,0);
        }
    }
}

// bool Wthread::videosave(cv::Mat &l, cv::Mat &r)
// {
    
// }