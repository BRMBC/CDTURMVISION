/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:01:24
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-22 00:28:39
 * @FilePath: /RMvision/extools/predictor.cpp
 * @Description: 装甲板位置预测
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#include "predictor.h"

#include<opencv4/opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Predictor::Predictor()
{
    FileStorage fs("/home/brmbc/桌面/qt-project/CDTURMvision/configure/camerasettings.xml", FileStorage::READ);
    Mat processNoise, measurementNoise;
    Mat t;
    fs["Kalman_Q"] >> processNoise;
    fs["Kalman_R"] >> measurementNoise;
    fs.release();

    KF = std::make_shared<cv::KalmanFilter>(state_num, measure_num, 0);

    cv::Mat H = (cv::Mat_<float>(measure_num, state_num) << 1, 0, 0, 0, 0, 0,
                                                            0, 0, 1, 0, 0, 0,
                                                            0, 0, 0, 0, 1, 0);  // H 测量矩阵

    KF->processNoiseCov = processNoise;                                         // Q 过程噪声
    KF->measurementNoiseCov = measurementNoise;                                 // R 测量噪声
    cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));                      // P 卡尔曼增益
    cv::setIdentity(KF->transitionMatrix, cv::Scalar::all(1));                  // F 状态转移矩阵
    KF->measurementMatrix = H;
}

void Predictor::initState(cv::Point3f ov, bool spin_once_flag)
{
    if(spin_once_flag)
    {
        KF -> statePost = (cv::Mat_<float>(state_num, 1) << ov.x,
                                                            KF -> statePost.at<float>(1, 0),
                                                            ov.y,
                                                            KF -> statePost.at<float>(3, 0),
                                                            ov.z,
                                                            KF -> statePost.at<float>(5, 0)
                                                            );
    }
    else
    {
        KF -> statePost = (cv::Mat_<float>(state_num, 1) <<  ov.x,
                                                             0,
                                                             ov.y,
                                                             0,
                                                             ov.z,
                                                             0
                                                             );
    }
}

cv::Point3f Predictor::predict(Armor &armor, cv::Point3f ov)
{

    cv::Mat measurement = (cv::Mat_<float>(measure_num,1) << ov.x,
                                                             ov.y,
                                                             ov.z
                                                            );
    t1 = cv::getTickCount();
    float dt = (t1 - armor.t)/cv::getTickFrequency();
    t1 = 0;
    KF->transitionMatrix.at<float>(0, 1) = dt;
    KF->transitionMatrix.at<float>(2, 3) = dt;
    KF->transitionMatrix.at<float>(4, 5) = dt;

    KF->predict();

    cv::Mat estimated = KF->correct(measurement);

    float pitch_gc = GC(ov);

    cv::Point3f av = next_point(armor, estimated, dt);

    return av;
}

float Predictor::GC(cv::Point3f xyz)
{
    xyz.y -= 60;
    float dy = 0, angle = 0, y_actual = 0;
    float t_actual = 0.0;
    float y_temp = -xyz.y / 1000.0;
    float y = y_temp;
    float x = sqrt(xyz.x * xyz.x + xyz.z * xyz.z) / 1000.0;
    for(int i=0; i<iter_num; i++)
    {
        angle = atan2(y_temp, x);
        t_actual = getflytime(angle, xyz);
        float v_y = bs_coeff * bullet_speed * sin(angle);
        y_actual = v_y * t_actual - g * t_actual * t_actual / 2.0;
        dy = y - y_actual;
        y_temp += dy;
        if(abs(dy)<0.001)
            break;
    }
    float pitch_gc = angle * 180.0 / CV_PI + 1.2;
    
    return pitch_gc;
}

cv::Point3f Predictor::next_point(Armor &armor, cv::Mat &result, float t)
{
    float x = result.at<float>(0, 0) + t * result.at<float>(1, 0);
    float y = result.at<float>(2, 0) + t * result.at<float>(3, 0);
    float z = result.at<float>(4, 0) + t * result.at<float>(5, 0);

    xyz = cv::Point3f(x, y, z);

    float yaw =  atan(x / z) * 180.0 / CV_PI;
    float pitch = atan(y / sqrt(x * x + z * z)) * 180.0 / CV_PI;
    float distance = sqrt(x * x + y * y + z * z) / 1000.0;

    return cv::Point3f(yaw, pitch, distance);
}
