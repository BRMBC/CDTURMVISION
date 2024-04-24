/*
 * @Author: BRMBC
 * @Date: 2024-03-13 15:12:08
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-22 04:16:45
 * @FilePath: /RMvision/Armor/spinDetector.cpp
 * @Description: 小陀螺判断
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
 
#include "spinDetector.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

void SpinDetector::reset()
{
    yp_window.clear();
    //armor_window.clear();
    ft = 0;
    lt = 0;
    spin_mode = 0;
    state = ArmorState::LOST;
}

bool SpinDetector::ifspin(std::vector<cv::Point2f> &yp_window)
{
    lt = cv::getTickCount();
    spin_t = (ft - lt) / cv::getTickFrequency();
    if(spin_t < 1.0)
    {
        float df = yp_window[0].x;
        float dl = yp_window.back().x;
        spin_yaw = (dl-df)/(lt-ft);
        spin_pitch = (yp_window[0].y + yp_window.back().y) / 2;
    }
    else
    {
        reset();
        return false;
    }
    return true;
}

bool SpinDetector::spinsolve(Armor &armor, std::vector<cv::Point2f> &yp_window)
{
    if(ifspin(yp_window))
    {
        armor.spin_ypt = cv::Point3f(spin_yaw, spin_pitch, spin_t);
        spin_mode = 1;
        return true;
    }
    else
    {
        spin_mode = 0;
        return false;
    }
    return true;
}

bool SpinDetector::run(Armor &armor)
{
    if(state == ArmorState::LOST && armor.state == ArmorState::FIRST)
    {
        yp_window.push_back(cv::Point2f(armor.ypd.x, armor.ypd.y));
        //armor_window.push_back(armor);
        ft = cv::getTickCount();
        state = armor.state;
        return true;
    }
    else if(state == ArmorState::FIRST && armor.state == ArmorState::SHOOT)
    {
        yp_window.push_back(cv::Point2f(armor.ypd.x, armor.ypd.y));
        state = armor.state;
        return true;
    }
    else if(state == ArmorState::FIRST && armor.state == ArmorState::FLOST)
    {
        return true;
    }
    else if(state == ArmorState::SHOOT && armor.state == ArmorState::SHOOT)
    {
        yp_window.push_back(cv::Point2f(armor.ypd.x, armor.ypd.y));
        state = armor.state;
        if(yp_window.size() > 30)
        {
            reset();
            return false;
        }
        return true;
    }
    else if(state == ArmorState::SHOOT && armor.state == ArmorState::FLOST)
    {
        return true;
    }
    else if(state == ArmorState::SHOOT && armor.state == ArmorState::LOST)
    {
        yp_window.push_back(cv::Point2f(armor.ypd.x, armor.ypd.y));
        state = armor.state;
        if(spinsolve(armor, yp_window))
        {
            armor.spin_ypt = cv::Point3f(spin_yaw, spin_pitch, spin_t);
            return true;
        }
        else
        {
            reset();
            return false;
        }
    }
    return true;
}

SpinDetector::SpinDetector()
{
    reset();
}
