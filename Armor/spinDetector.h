/*
 * @Author: BRMBC
 * @Date: 2024-03-13 15:16:34
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-22 00:44:36
 * @FilePath: /RMvision/Armor/spinDetector.h
 * @Description: 小陀螺判断
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#ifndef SPINDETECTOR_H
#define SPINDETECTOR_H

#include"armorBox.h"
#include"tools.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>


class SpinDetector 
{
    public:
        std::vector<cv::Point2f>yp_window;              //yaw-pitch数据链
        float ft = 0;                                   //第一次装甲板
        float lt = 0;                                   //最后一次装甲板
        bool spin_mode = 0;                             //小陀螺模式
        ArmorState state = ArmorState::LOST;            //识别状态
        float spin_yaw = 0;                             //小陀螺yaw变化均值
        float spin_pitch = 0;                           //小陀螺pitch变化均值
        float spin_t = 0;                               //小陀螺时间
    public:
        void reset();
        bool ifspin(std::vector<cv::Point2f>& yp_window);
        bool spinsolve(Armor &armor, std::vector<cv::Point2f>& yp_window);
        bool run(Armor &armor);

        SpinDetector();
        ~SpinDetector()=default;
};
#endif