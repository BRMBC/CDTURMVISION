/*
 * @Author: BRMBC
 * @Date: 2024-03-10 17:59:58
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:24:14
 * @FilePath: /RMvision/Armor/armorDetector.h
 * @Description: 装甲板识别
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */
#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include"tools.h"
#include"solver.h"
#include"predictor.h"
#include"armorClassifier.h"
#include"armorBox.h"

#include<iostream>
#include<cmath>
#include<vector>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/imgproc.hpp>

class ArmorDetector
{
private:
    Classifier classifier = Classifier("../configure/param/");
    Solver solve;                                                                    //PNP解算
    Predictor predict;                                                               //Kalman预测
    GC gc;                                                                           //重力补偿算法
    DebugParam debug;                                                                //调试参数
    SpinDetector Lspindetector, Rspindetector;                                       //小陀螺判断
    int lost_count = 0;                                                              //丢失次数
    cv::Mat Lgray, Rgray;                                                            //灰度图像
    cv::Mat Lsrc, Rsrc;                                                              //处理图像
    cv::Mat Lbgr, Rbgr;                                                              //BGR颜色处理图像
    std::vector<cv::Mat> Lchannels, Rchannels;                                       //BGR通道分离图像
    cv::Mat LthresHold1, LthresHold2, RthresHold1, RthresHold2;                      //二值化处理图像
    cv::Point2f Lpoint[2], Rpoint[2];                                                //装甲板坐标点
    cv::Point3f Lypd = cv::Point3f(0, 0, 0), Rypd = cv::Point3f(0, 0, 0);            //装甲板坐标值, Rypd;                                                          //上次识别数据
    char Llast_state = 0, Rlast_state = 0;                                           //识别状态
    int L_lost_count = 0, R_lost_count = 0;                                          //未识别次数
    cv::Point3f Llost_xyz, Rlost_xyz;                                                //上次识别坐标值
    bool spin_mode = 0;                                                              //小陀螺模式
private:
    /**
     * @brief 获取图像
     * @param 将相机获取的图像img传给light,armor
    */
    bool originalimg(Armor &armor);
    /**
     * @brief 左-寻找灯条
     * @param 识别的灯条light_Rec存入lightcontour
    */
    bool Lfindlight(Armor &armor);
    /**
     * @brief 左-寻找装甲板
     * @param 得出正确的装甲板灯条设置该装甲板的左右灯条l,r
    */
    bool Lfindarmor(Armor &armor);
    /**
     * @brief 左-得出目标装甲板
     * @param
    */
    bool Lmatcharmor(Armor &armor);
    /**
     * @brief 左-预测状态判断
     * @param
    */
    bool LStateDetection(Armor &armor);
    /**
     * @brief 右-寻找灯条
     * @param 识别的灯条light_Rec存入lightcontour
    */
    bool Rfindlight(Armor &armor);
    /**
     * @brief 右-寻找装甲板
     * @param 得出正确的装甲板灯条设置该装甲板的左右灯条l,r
    */
    bool Rfindarmor(Armor &armor);
    /**
     * @brief 左-得出目标装甲板
     * @param
    */
    bool Rmatcharmor(Armor &armor);
    /**
     * @brief 右-预测状态判断
     * @param
    */
    bool RStateDetection(Armor &armor);
    /**
    * @brief 设置正确的PNP解算点
    * @param 顺序：左上-右上-右下-左下
    */
    bool PnPget(Armor &armor);
    /**
    * @brief 判断是否为同一装甲板灯条
    * @param llightcontour 左灯条
    * @param rlightcontour 右灯条
    */
    bool isCoupleLight(cv::RotatedRect &llightcontour, cv::RotatedRect &rlightcontour);
    /**
     * @brief 判断装甲板类型
     * @param armor 装甲板类
     * @param llightcontour 左灯条
     * @param rlightcontour 右灯条
    */
    bool ArmorBox(Armor &armor, size_t &i, size_t &j);
    /**
     * @brief 灯条位置预处理
     * @param rec 需要调整的灯条旋转矩形
     * @param mode 调整模式
    */
    cv::RotatedRect &adjustlight(cv::RotatedRect &rec, const int mode);
    /**
     * @brief 装甲板位置预处理
     * @param img 输入图像
     * @param box 装甲板区域矩形
    */
    void adjustarmor(cv::Rect &box, cv::Mat &img);
    /**
     * @brief 装甲板区域像素处理
     * @param src 输入图像
     * @param dst 输出图像
     * @param fgamma 伽马值
    */
    void Gamma(cv::Mat &src, cv::Mat &dst, float fgamma);
    /**
     * @brief 装甲板数字识别
     * @param armor 装甲板类
    */
    bool getArmorNum(Armor &armor);


public:
    /**
     * @brief 左-对外API
     * @param
    */
    bool LRun(Armor &armor);
    /**
     * @brief 右-对外API
     * @param
    */
    bool RRun(Armor &armor);
};

#endif // ARMORDETECTOR_H
