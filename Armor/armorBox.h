/*
 * @Author: brmbc
 * @Date: 2024-03-10 17:59:55
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-22 00:12:17
 * @FilePath: /RMvision/Armor/armorBox.h
 * @Description: Armor类，装甲板的各种参数
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#ifndef ARMORBOX_H
#define ARMORBOX_H

#include"tools.h"
#include<iostream>
#include<cmath>
#include<vector>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/imgproc.hpp>


//装甲板参数
class Armor
{
public:
    cv::Mat img;                                                    //传入图像
    cv::Mat light;                                                  //灯条图像
    cv::Mat armor;                                                  //装甲板图像
    cv::Mat ROI;                                                    //装甲板区域图像
    cv::Mat NUM;                                                    //装甲板数字图像
    cv::RotatedRect rect;                                           //装甲板旋转矩形
    cv::Rect box;                                                   //装甲板正接矩形
    std::vector<cv::Rect>boxes;                                     //装甲板旋转矩形
    std::vector<cv::RotatedRect> rects;                             //装甲板矩形
    std::vector<cv::RotatedRect> lightcontour;                      //储存灯条矩形
    std::vector<cv::Point2f> pnp;                                   //用于PNP解算的坐标点
    cv::Point3f ypd = cv::Point3f(0, 0, 0);                         //yaw\pitch\distance;
    cv::Point3f pxyz = cv::Point3f(0, 0, 0);                        //预测坐标                                              //经过Kalman滤波后的yaw\pitch\distance数据
    cv::Point2f p[4];                                               //装甲板四个顶点的坐标
    cv::Point2f l[4];                                               //左边灯条的坐标
    cv::Point2f r[4];                                               //右边灯条的坐标
    float t = 0;
    int id = 0;
    ArmorType armortype;                                            //装甲板类型
    ArmorState state = ArmorState::LOST;                            //预测计算实时状态
    char found = 0;                                                 //装甲板识别状态
    char fire = 0;                                                  //
    float pitch = 0.0;                                              //电机俯仰角
    cv::Point3f spin_ypt = cv::Point3f(0, 0, 0);                    //小陀螺yaw-pitch变化
};

#endif // ARMORBOX_H
