/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:01:26
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-23 19:11:13
 * @FilePath: /RMvision/extools/tools.h
 * @Description: 
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#ifndef EXTOOLS_H
#define EXTOOLS_H

#include<vector>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/opencv.hpp>

class GC
{
private:
    double g = 9.75;                              //重力加速度
    double k = 0.00402;                           //阻力系数
    int iter_num = 30;                            //迭代次数
    float bullet_speed = 23.0;                    //弹速
    float bs_coeff = 0.9;                         //速度系数

private:
    float getflytime(double angle, cv::Point3f xyz);

public:
    float compensate(cv::Point3f xyz);
};


class DebugParam
{
public:
    bool debug_img_show = 1;                      //图像显示模式
    bool debug_mode = 0;                          //debug模式
    bool ldebug_mode = 0;                         //二值化显示模式
    bool rdebug_mode = 0;                         //二值化显示模式
    bool debug_llight_show = 0;                   //左灯条显示模式
    bool debug_larmor_show = 0;                   //左装甲板显示模式
    bool debug_rlight_show = 0;                   //右灯条显示模式
    bool debug_rarmor_show = 0;                   //右装甲板显示模式
    bool debug_light_error_print = 0;             //灯条错误显示模式
    bool debug_armor_error_print = 0;             //装甲板错误显示模式
    bool debug_send_data = 1;                     //串口数据显示模式
    bool debug_data_write = 0;                    //数据写入模式
};
//识别颜色
enum Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};

enum ArmorType
{
    SMALL = 0,
    BIG = 1,
    RUNE_ARMOR = 2
};

enum ArmorState
{
    LOST = 0,       // 丢失目标
    FIRST = 1,      // 第一次发现目标
    SHOOT = 2,      // 持续识别目标
    FLOST = 3       // 识别到第一次丢失
};

enum Mode
{
    NORMAL = 1,     // 普通模式
    RUNE = 2,       // 大符模式
    NORMAL_RUNE = 3 // 小符模式
};

#endif // EXTOOLS_H
