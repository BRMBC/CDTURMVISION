/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:01:26
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:36:34
 * @FilePath: /RMvision/extools/solver.h
 * @Description: PNP解算
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */
#ifndef SOLVER_H
#define SOLVER_H

#include"tools.h"

#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include<fstream>


// 解算参数
typedef struct SolverParam
{
    cv::Mat lcamera_matrix;               // 左相机内参矩阵
    cv::Mat ldist_coeffs;                 // 左相机畸变系数
    cv::Mat rcamera_matrix;               // 左相机内参矩阵
    cv::Mat rdist_coeffs;                 // 左相机畸变系数
    float small_armor_boxes_real_height; // 小装甲板的高
    float small_armor_boxes_real_width;  // 小装甲板的宽
    float big_armor_boxes_real_height;   // 大装甲板的高
    float big_armor_boxes_real_width;    // 大装甲板的宽
    float rune_armor_boxes_real_height;  // 能量机关装甲板的高
    float rune_armor_boxes_real_width;   // 能量机关装甲板的宽
    SolverParam();
} SolverParam;

class Solver
{
private:
    std::vector<cv::Point3f> Lpoints_3D;  // 装甲板世界坐标（人为设置）
    std::vector<cv::Point3f> Rpoints_3D;  // 装甲板世界坐标（人为设置）


public:
    SolverParam param;                                         // 解算参数
    GC lgc, rgc;                                               // 重力补偿算法

    float lpitch = 0;                                          // pitch俯仰角(y/z)
    float lyaw = 0;                                            // yaw侧航角(x/z)
    float ldistance = 0;                                       // 距离
    cv::Mat lrVec;                                             // 旋转矩阵
    cv::Mat ltVec;                                             // 平移矩阵
    cv::Point3f lpoint_3D = cv::Point3f(0, 0, 0);              // 相机坐标系下目标点
    cv::Point3f lypd = cv::Point3f(0, 0, 0);

    float rpitch = 0;                                          // pitch俯仰角(y/z)
    float ryaw = 0;                                            // yaw侧航角(x/z)
    float rdistance = 0;                                       // 距离
    cv::Mat rrVec;                                             // 旋转矩阵
    cv::Mat rtVec;                                             // 平移矩阵
    cv::Point3f rpoint_3D = cv::Point3f(0, 0, 0);              // 相机坐标系下目标点
    cv::Point3f rypd = cv::Point3f(0, 0, 0);

    typedef ArmorType BoxSize;

    BoxSize Lbox_size;
    BoxSize Rbox_size;
private:

    /**
     * @brief 设置3D点集
     * @param _box_size  装甲板的类型
     */
    void Lset3DPoints(const BoxSize &_box_size);

    void Rset3DPoints(const BoxSize &_box_size);

public:
    /**
     * 函数名称： solve(const vector<Point2f> &_points_2D ,const BoxSize &_box_size)
     * 功能描述： 进行PNP解算
     * 参数说明:  _points_2D -> 进行PNP解算的四个角点
     *          _box_size ->  装甲板的类型，不明确时，默认值为小装甲板
     * 返回值：无
    */
    bool Lsolve(const std::vector<cv::Point2f> &point_2D, const BoxSize &_box_size = SMALL);

    bool Rsolve(const std::vector<cv::Point2f> &point_2D, const BoxSize &_box_size = SMALL);

    Solver() = default;
};

#endif // SOLVER_H
