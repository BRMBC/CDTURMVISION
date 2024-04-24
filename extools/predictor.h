/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:01:24
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-17 11:42:22
 * @FilePath: /RMvision/extools/predictor.h
 * @Description: 装甲板位置预测
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#ifndef PREDICTOR_H
#define PREDICTOR_H

#include<spinDetector.h>
#include<armorBox.h>

#include <eigen3/Eigen/Dense>
#include <chrono>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>

class Predictor
{
private:
    std::shared_ptr<cv::KalmanFilter> KF;
    const int measure_num = 3;
    const int state_num = 6;
    //float dt = 0.012;                       //时间变化

    float bullet_speed = 23.0;              //子弹射速
    float delay_time = 0.009;               //发弹延迟、通信延迟等较为固定延迟
    double k = 0.0402;                      //阻力系数
    float g = 9.75;                         //重力加速度
    int iter_num = 40;                      //迭代次数
    float t1 = 0;                           //时间戳

    double bs_coeff = 0.9;                  //初始弹速系数

public:
    cv::Point3f xyz = cv::Point3f(0, 0, 0);                        //识别预测位置
    cv::Point3f ypd = cv::Point3f(0, 0, 0);                        //识别预测姿态

public:
    /**
     * @brief
     * @param
     * @return
    */
    void initState(cv::Point3f ov, bool spin_once_flag = false);
    /**
     * @brief
     * @param
     * @return
    */
    cv::Point3f predict(Armor &armor, cv::Point3f ov);
    /**
     * @brief 重力和时间补偿
     * @param
     * @return 返回计算过后的x,y,z值
    */
    float GC(cv::Point3f xyz);
    /**
     * @brief 计算飞行时间
     * @param angle 飞行角度
     * @param xyz 世界坐标点
     * @return 飞行时间
    */
    float getflytime(float angle, cv::Point3f xyz)
    {
        double x = sqrt(xyz.x * xyz.x + xyz.z * xyz.z) / 1000.0;
        float v_x = bs_coeff * bullet_speed * cos(angle);
        float t_x = (exp(k*x) - 1.0) / (k*v_x);
        return t_x;
    }
    cv::Point3f next_point(Armor &armor, cv::Mat &result, float t);

    Predictor();
    ~Predictor() = default;
};

#endif // PREDICTOR_H
