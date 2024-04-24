/*
 * @Author: BRMBC
 * @Date: 2024-03-10 18:01:26
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:36:18
 * @FilePath: /RMvision/extools/solver.cpp
 * @Description: PNP解算
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */
#include"solver.h"
#include<vector>

using namespace cv;
using namespace std;

//float ldpitch, rdpitch;

bool Solver::Lsolve(const vector<Point2f> &point_2D, const BoxSize &_box_size)
{

    //ofstream file("../configure/ldata.txt", ios::app);
    lyaw = 0;
    lpitch = 0;
    ldistance = 0;
    Lset3DPoints(_box_size);
    solvePnP(Lpoints_3D, point_2D, param.lcamera_matrix, param.ldist_coeffs, lrVec, ltVec, false, SOLVEPNP_IPPE_SQUARE);
    double x = ltVec.at<double>(0, 0), y = ltVec.at<double>(1, 0), z = ltVec.at<double>(2, 0);
    if(z < 0)
        return false;
    lpoint_3D = Point3f((float)x, (float)y, (float)z);
    //cout<<"point:"<<lpoint_3D<<endl;
    float lpitch_gc = lgc.compensate(lpoint_3D);
    lyaw = atan(x / z) * 180.0 / CV_PI;
    if(std::isnan(lyaw))
        return false;
    lpitch = atan(-y / z) * 180.0 / CV_PI;
    if(std::isnan(lpitch))
        return false;
    ldistance = sqrt(x*x + y*y + z*z) / 1000.0;
    if(std::isnan(ldistance) || ldistance < 0)
        return false;
    // if (file.is_open()) 
    // {
    //     file << lyaw <<" "<< lpitch <<" "<< ldistance << endl;
    //     file.close();
    //     //std::cout << "Float value appended to file successfully." << std::endl;
    // }
    //cout<<"左原pitch："<<lpitch<<" 重力补偿pitch："<<lpitch_gc<<endl;
    //cout<<"左原yaw:"<<lyaw<<" pitch:"<<lpitch<<" distance:"<<ldistance<<endl;
    lypd = Point3f(lyaw, lpitch_gc, ldistance);
    return true;
}

bool Solver::Rsolve(const vector<Point2f> &point_2D, const BoxSize &_box_size)
{
    ryaw = 0;
    rpitch = 0;
    rdistance = 0;
    Rset3DPoints(_box_size);
    solvePnP(Rpoints_3D, point_2D, param.lcamera_matrix, param.ldist_coeffs, rrVec, rtVec, false, SOLVEPNP_IPPE_SQUARE);
    double x = rtVec.at<double>(0, 0), y = rtVec.at<double>(1, 0), z = rtVec.at<double>(2, 0);
    if(z < 0)
        return false;
    rpoint_3D = Point3f((float)x, (float)y, (float)z);
    float rpitch_gc = rgc.compensate(rpoint_3D);
    //cout<<"point:"<<lpoint_3D<<endl;
    ryaw =   atan(x / z) * 180.0 / CV_PI;
    if(std::isnan(ryaw))
        return false;
    rpitch = atan(-y / z) * 180.0 / CV_PI;
    if(std::isnan(rpitch))
        return false;
    rdistance = sqrt(x*x + y*y + z*z) / 1000.0;
    if(std::isnan(rdistance) || rdistance < 0)
        return false;
    if(std::isnan(ryaw + rpitch_gc + rdistance))
    {
        //rypd = rypd;
        return false;
    }
    //cout<<"电机角度"<<ldpitch<<endl;
    // if(rdpitch > -4 && rpitch_gc > 6)
    // {
    //     //cout<<"窗户位置"<<endl;
    //     //rypd =rypd;
    //     return false;
    // }
    //cout<<"右原pitch："<<rpitch<<" 重力补偿pitch："<<rpitch_gc<<endl;
    //cout<<"右原yaw:"<<ryaw<<" pitch:"<<rpitch<<" distance:"<<rdistance<<endl;
    rypd = Point3f(ryaw, rpitch_gc, rdistance);

    return true;
}

/**
 * 函数名称： set3DPoints(const BoxSize &_box_size)
 * 功能描述： 设置3D点集
 * 参数说明： _box_size -> 装甲板的类型
 * 返回值： 无
*/
void Solver::Lset3DPoints(const BoxSize &_box_size)
{
    if(Lbox_size==_box_size &&!Lpoints_3D.empty())
        return ;
    else
    {
        Lpoints_3D.clear();
        int width = 0, height = 0;
        Lbox_size=_box_size;
        if(Lbox_size == BIG)
        {
            width=param.big_armor_boxes_real_width;
            height=param.big_armor_boxes_real_height;
        }
        else if(Lbox_size == SMALL)
        {
            width=param.small_armor_boxes_real_width;
            height=param.small_armor_boxes_real_height;
        }
        else if (Lbox_size == RUNE_ARMOR)
        {
            width = param.rune_armor_boxes_real_width;
            height = param.rune_armor_boxes_real_height;
        }
        Lpoints_3D.push_back(Point3f(-width/2.0, -height/2.0, 0));//左上
        Lpoints_3D.push_back(Point3f( width/2.0, -height/2.0, 0));//右上
        Lpoints_3D.push_back(Point3f( width/2.0,  height/2.0, 0));//右下
        Lpoints_3D.push_back(Point3f(-width/2.0,  height/2.0, 0));//左下
    }
}

void Solver::Rset3DPoints(const BoxSize &_box_size)
{
    if(Rbox_size==_box_size &&!Rpoints_3D.empty())
        return ;
    else
    {
        Rpoints_3D.clear();
        int width = 0, height = 0;
        Rbox_size=_box_size;
        if(Rbox_size == BIG)
        {
            width=param.big_armor_boxes_real_width;
            height=param.big_armor_boxes_real_height;
        }
        else if(Rbox_size == SMALL)
        {
            width=param.small_armor_boxes_real_width;
            height=param.small_armor_boxes_real_height;
        }
        else if (Rbox_size == RUNE_ARMOR)
        {
            width = param.rune_armor_boxes_real_width;
            height = param.rune_armor_boxes_real_height;
        }
        Rpoints_3D.push_back(Point3f(-width/2.0, -height/2.0, 0));//左上
        Rpoints_3D.push_back(Point3f( width/2.0, -height/2.0, 0));//右上
        Rpoints_3D.push_back(Point3f( width/2.0,  height/2.0, 0));//右下
        Rpoints_3D.push_back(Point3f(-width/2.0,  height/2.0, 0));//左下
    }
}

/**
 * 函数名称： SolverParam()
 * 功能描述： 传入使用参数
 * 参数说明： None
 * 返回值： 无
*/
SolverParam::SolverParam()
{

    FileStorage fs("../configure/camerasettings.xml", FileStorage::READ);
    if(!fs.isOpened())
    {
        cout<<"PNP参数加载错误"<<endl;
    }
    fs["lcamera_matrix"] >> lcamera_matrix;
    fs["rcamera_matrix"] >> rcamera_matrix;
    cout<<"lcamera_matrix"<<lcamera_matrix<<"\n"<<"rcamera_matrix"<<rcamera_matrix<<endl;
    fs["ldist_coeffs"] >> ldist_coeffs;
    fs["rdist_coeffs"] >> rdist_coeffs;
    cout<<"ldist_coeffs"<<ldist_coeffs<<"\n"<<"rdist_coeffs"<<rdist_coeffs<<endl;
    fs["small_armor_boxes_real_height"] >> small_armor_boxes_real_height;
    fs["small_armor_boxes_real_width"] >> small_armor_boxes_real_width;
    fs["big_armor_boxes_real_height"] >> big_armor_boxes_real_height;
    fs["big_armor_boxes_real_width"] >> big_armor_boxes_real_width;
    fs["rune_armor_boxes_real_height"] >> rune_armor_boxes_real_height;
    fs["rune_armor_boxes_real_width"] >> rune_armor_boxes_real_width;

    fs.release();
}
