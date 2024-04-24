/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:01:26
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-03-19 17:21:07
 * @FilePath: /RMvision/extools/tools.cpp
 * @Description: 
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#include "tools.h"

using namespace cv;
using namespace std;

float GC::getflytime(double angle, Point3f xyz)
{
    double x = sqrt(xyz.x * xyz.x + xyz.z * xyz.z) / 1000.0;
    float v_x = bs_coeff * bullet_speed * cos(angle);
    float t_x = (exp(k*x) - 1.0) / (k*v_x);
    return t_x;
}
float GC::compensate(Point3f xyz)
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