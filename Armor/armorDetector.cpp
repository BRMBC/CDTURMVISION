/*
 * @Author: BRMBC
 * @Date: 2024-03-10 17:59:58
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:23:32
 * @FilePath: /RMvision/Armor/armorDetector.cpp
 * @Description: 装甲板识别
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */
#include"armorDetector.h"
#include"tools.h"

extern Color ENEMYCOLOR;                                    //定义敌方颜色参数，在mian.cpp修改
const int ANGLE_TO_UP = 1;                                  //角度
const int WIDTH_GREATER_THAN_HEIGHT = 0;                    //长宽比

using namespace cv;
using namespace std;

bool ArmorDetector::originalimg(Armor &armor)
{
    armor.img.copyTo(armor.light);
    armor.img.copyTo(armor.armor);
    if(armor.light.empty())
        return false;
    return true;
}

bool ArmorDetector::Lfindlight(Armor &armor)
{
    armor.lightcontour.clear();
    if(armor.light.empty())
    {
        cout<<"no imginput"<<endl;
        return false;
    }
    //使用BGR通道(OPENCV处理图像方式)
    cvtColor(armor.light, Lgray, COLOR_BGR2GRAY);
    if(ENEMYCOLOR == 2)//红色
    {
        threshold(Lgray, LthresHold1, 100, 255, THRESH_BINARY);
        split(armor.light, Lchannels);
        Lbgr = Lchannels.at(2) - Lchannels.at(0);//R通道-B通道
        threshold(Lbgr, LthresHold2, 25, 255, THRESH_BINARY);
        Lsrc = LthresHold1 & LthresHold2;
    }
    else if(ENEMYCOLOR == 0)//蓝色
    {
        threshold(Lgray, LthresHold1, 100, 255, THRESH_BINARY);
        split(armor.light, Lchannels);
        Lbgr = Lchannels.at(0) - Lchannels.at(2);//B通道-R通道
        threshold(Lbgr, LthresHold2, 25, 255, THRESH_BINARY);
        Lsrc = LthresHold1 & LthresHold2;
    }
    Mat Lelement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));                        //创建结构元素用于形态学操作
    morphologyEx(Lsrc, Lsrc, MORPH_CLOSE, Lelement);                                       //拓展灯条效果
    vector<vector<Point>>light_contour;                                                    //疑似灯条坐标点
    findContours(Lsrc, light_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);                 //查找灯条轮廓点传入light_contour
    for(size_t i = 0; i < light_contour.size(); i++)
    {
        if(contourArea(light_contour[i]) < 10 || contourArea(light_contour[i]) > 5500)
        {
            if(debug.debug_light_error_print)
                cout<<"L灯条面积过小或过大"<<contourArea(light_contour[i])<<endl;
            continue;
        }
        float light_contour_area = contourArea(light_contour[i]);
        RotatedRect light_Rec = fitEllipse(light_contour[i]);//椭圆拟合
        light_Rec = adjustlight(light_Rec, ANGLE_TO_UP);
        if(light_Rec.angle > 25)
        {
            if(debug.debug_light_error_print)
                cout<<"L灯条角度过大"<<light_Rec.angle<<endl;
            continue;
        }
        if(light_Rec.size.height / light_Rec.size.width < 1.5 ||
           light_contour_area    / light_Rec.size.area() < 0.44)
        {
            if(debug.debug_light_error_print)
                cout<<"L灯条长宽比过小"<<endl;
            continue;
        }
        light_Rec.size.height*=0.80;
        light_Rec.size.width*=0.90;
        armor.lightcontour.push_back(light_Rec);
        //显示灯条
        if(debug.debug_llight_show)
        {
            Point2f d[4];
            light_Rec.points(d);
            for(int a = 0; a < 4; a++)
            {
                line(armor.light, d[a], d[(a+1)%4], Scalar(255,255,255), 1);
            }
        }
    }
    if(debug.debug_llight_show)
    {
        imshow("Llight", armor.light);
        if(debug.ldebug_mode)
        {
            imshow("左相机二值化处理结果1", LthresHold1);
            imshow("左相机二值化处理结果2", LthresHold2);
            imshow("L灯条效果", Lsrc);

        }
    }
    if(armor.lightcontour.size() < 2)
    {
         //cout<<armor.lightcontour.size()<<endl;
         return false;
    }
    return true;
}

bool ArmorDetector::Lfindarmor(Armor &armor)
{
    auto cmp = [](RotatedRect a, RotatedRect b) -> bool
    {
        return a.center.x < b.center.x;
    };
    sort(armor.lightcontour.begin(), armor.lightcontour.end(), cmp);
    armor.rects.clear();
    for(size_t i = 0; i < armor.lightcontour.size() - 1; i++)
    {
        for(size_t j = i + 1; j < armor.lightcontour.size(); j++)
        {
            if(!isCoupleLight(armor.lightcontour[i], armor.lightcontour[j]))
                continue;
            if(!ArmorBox(armor, i, j))
                continue;
            if(armor.rects.size() > 1)
                continue;
            armor.rects.push_back(armor.rect);
            armor.lightcontour[i].points(armor.l);
            armor.lightcontour[j].points(armor.r);
            armor.rects[0].points(armor.p);
            if(debug.debug_larmor_show)
            {
                for(int i = 0; i < 4; i++)
                {
                    line(armor.armor, armor.p[i], armor.p[(i+1)%4], Scalar(255,255,255), 1);
                }
            }
        }
    }
    //cout<<"armor.rects.size:"<<armor.rects.size()<<endl;
    if (armor.rects.size() < 1)
    {
        return false;
    }
     //getArmorNum(armor);
    if(debug.debug_larmor_show)
    {
//        imshow("lROI", armor.ROI);
        //imshow("LUNM", armor.NUM);
        circle(armor.armor, armor.rects[0].center, 2, Scalar(0, 255, 0), -1);
        circle(armor.armor, Point(640, 512), 2, Scalar(0, 250, 0), -1);
        imshow("Larmor", armor.armor);
    }
    return true;
}

bool ArmorDetector::LStateDetection(Armor &armor)
{
    if(Llast_state == 0 && armor.found == 1 && L_lost_count == 0)
    {
        armor.state = ArmorState::FIRST;
        Llast_state = armor.found;
        return true;
    }
    else if(Llast_state == 0 && armor.found == 1 && L_lost_count > 0)
    {
        L_lost_count = 0;
        armor.state = ArmorState::SHOOT;
        Llast_state = armor.found;
        return true;
    }
    if(Llast_state == 1 && armor.found == 1)
    {
        armor.state = ArmorState::SHOOT;
        Llast_state = armor.found;
        return true;
    }
    if(Llast_state == 1 && armor.found == 0)
    {
        L_lost_count = 1;
        armor.state = ArmorState::FLOST;
        Llast_state = armor.found;
        //armor.found = 1;
        return true;
    }
    if(Llast_state == 0 && armor.found == 0 && L_lost_count > 0 && L_lost_count <= 3)
    {
        L_lost_count += 1;
        armor.state = ArmorState::FLOST;
        Llast_state = armor.found;
        //armor.found = 1;
        return true;
    }
    else if(Llast_state == 0 && armor.found == 0 && L_lost_count > 3)
    {
        L_lost_count = 0;
        armor.state = ArmorState::LOST;
        Llast_state = armor.found;
        return false;
    }
    else if(Llast_state == 0 && armor.found == 0 && L_lost_count == 0)
    {
        armor.state = ArmorState::LOST;
        Llast_state = armor.found;
        return false;
    }
    return true;
}

bool ArmorDetector::Lmatcharmor(Armor &armor)
{
    if(!originalimg(armor))
        return false;
    if(!Lfindlight(armor))
    {
        //cout<<"LLIGHT"<<endl;
        return false;
    }
    if(!Lfindarmor(armor))
    {
        //cout<<"LARMOR"<<endl;
        return false;
    }
    if(!PnPget(armor))
    {
        //cout<<"LPnP"<<endl;
        return false;
    }
    return true;
}

bool ArmorDetector::Rfindlight(Armor &armor)
{
    armor.lightcontour.clear();
    if(armor.light.empty())
    {
        cout<<"no imginput"<<endl;
        return false;
    }
    //使用BGR通道(OPENCV处理图像方式)
    cvtColor(armor.light, Rgray, COLOR_BGR2GRAY);
    if(ENEMYCOLOR == 2)//红色
    {
        threshold(Rgray, RthresHold1, 100, 255, THRESH_BINARY);
        split(armor.light, Rchannels);
        Rbgr = Rchannels.at(2) - Rchannels.at(0);//R通道-B通道
        threshold(Rbgr, RthresHold2, 25, 255, THRESH_BINARY);
        Rsrc = RthresHold1 & RthresHold2;
    }
    else if(ENEMYCOLOR == 0)//蓝色
    {
        threshold(Rgray, RthresHold1, 100, 255, THRESH_BINARY);
        split(armor.light, Rchannels);
        Rbgr = Rchannels.at(0) - Rchannels.at(2);//B通道-R通道
        threshold(Rbgr, RthresHold2, 25, 255, THRESH_BINARY);
        Rsrc = RthresHold1 & RthresHold2;
    }
    Mat Relement = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));                       //创建结构元素用于形态学操作
    morphologyEx(Rsrc, Rsrc, MORPH_CLOSE, Relement);                                       //拓展灯条效果
    vector<vector<Point>>light_contour;                                                    //疑似灯条坐标点
    findContours(Rsrc, light_contour, RETR_EXTERNAL, CHAIN_APPROX_NONE);                   //查找灯条轮廓点传入light_contour
    for(size_t i = 0; i < light_contour.size(); i++)
    {
        if(contourArea(light_contour[i]) < 10 || contourArea(light_contour[i]) > 5500)
        {
            if(debug.debug_light_error_print)
                cout<<"R灯条面积过小或过大:"<<contourArea(light_contour[i])<<endl;
                continue;
        }
        float light_contour_area = contourArea(light_contour[i]);
        RotatedRect light_Rec = fitEllipse(light_contour[i]);//椭圆拟合
        light_Rec = adjustlight(light_Rec,ANGLE_TO_UP);
        if(light_Rec.angle > 25)
        {
            if(debug.debug_light_error_print)
                cout<<"R灯条角度过大:"<<light_Rec.angle<<endl;
            continue;
        }
        if(light_Rec.size.height / light_Rec.size.width < 1.5 ||
           light_contour_area   / light_Rec.size.area() < 0.44)
        {
            if(debug.debug_light_error_print)
                cout<<"R灯条长宽比过小:"<<endl;
            continue;    
        }
        light_Rec.size.height*=0.80;
        light_Rec.size.width*=0.90;
        armor.lightcontour.push_back(light_Rec);
        //显示灯条(待修该if判断)
        if(debug.debug_rlight_show)
        {
            Point2f d[4];
            light_Rec.points(d);
            for(int a = 0; a < 4; a++)
            {
                line(armor.light, d[a], d[(a+1)%4], Scalar(255,255,255), 1);
            }
        }
    }
    if(debug.debug_rlight_show)
    {
        imshow("Rlight", armor.light);
        if(debug.rdebug_mode)
        {
        imshow("右相机二值化处理结果1", RthresHold1);
        imshow("右相机二值化处理结果2", RthresHold2);
        imshow("R灯条效果", Rsrc);

        }
    }
    if(armor.lightcontour.size() < 2)
        return false;
    return true;
}

bool ArmorDetector::Rfindarmor(Armor &armor)
{
    auto cmp = [](RotatedRect a, RotatedRect b) -> bool
    {
        return a.center.x < b.center.x;
    };
    sort(armor.lightcontour.begin(), armor.lightcontour.end(), cmp);
    armor.rects.clear();
    for(size_t i = 0; i < armor.lightcontour.size() - 1; i++)
    {
        for(size_t j = i + 1; j < armor.lightcontour.size(); j++)
        {
            if(!isCoupleLight(armor.lightcontour[i], armor.lightcontour[j]))
                continue;
            if(!ArmorBox(armor, i, j))
                continue;
            if(armor.rects.size() > 1)
                continue;
            armor.rects.push_back(armor.rect);
            armor.lightcontour[i].points(armor.l);
            armor.lightcontour[j].points(armor.r);
            armor.rects[0].points(armor.p);
            if(debug.debug_rarmor_show)
            {
                for(int i = 0; i < 4; i++)
                {
                    line(armor.armor, armor.p[i], armor.p[(i+1)%4], Scalar(255,255,255), 1);
                }
            }
        }
    }
    if(armor.rects.size() <= 0)
    {
        return false;
    }
    //getArmorNum(armor);
    if(debug.debug_rarmor_show)
    {
//        imshow("rROI", armor.ROI);
        //imshow("NUM", armor.NUM);
        circle(armor.armor, armor.rects[0].center, 2, Scalar(1, 255, 1), -1);
        circle(armor.armor, Point(640, 512), 2, Scalar(0, 250, 0), -1);
        imshow("Rarmor", armor.armor);
    }
    return true;
}

bool ArmorDetector::RStateDetection(Armor &armor)
{
    if(Rlast_state == 0 && armor.found == 1 && R_lost_count == 0)
    {
        armor.state = ArmorState::FIRST;
        Rlast_state = armor.found;
        return true;
    }
    else if(Rlast_state == 0 && armor.found == 1 && R_lost_count > 0)
    {
        R_lost_count = 0;
        armor.state = ArmorState::SHOOT;
        Rlast_state = armor.found;
        return true;
    }
    if(Rlast_state == 1 && armor.found == 1)
    {
        armor.state = ArmorState::SHOOT;
        Rlast_state = armor.found;
        return true;
    }
    if(Rlast_state == 1 && armor.found == 0)
    {
        R_lost_count = 1;
        armor.state = ArmorState::FLOST;
        Rlast_state = armor.found;
        //armor.found = 1;
        return true;
    }
    if(Rlast_state == 0 && armor.found == 0 && R_lost_count > 0 && R_lost_count <= 3)
    {
        R_lost_count += 1;
        armor.state = ArmorState::FLOST;
        Rlast_state = armor.found;
        //armor.found = 1;
        return true;
    }
    if(Rlast_state == 0 && armor.found == 0 && R_lost_count > 3)
    {
        R_lost_count = 0;
        armor.state = ArmorState::LOST;
        Rlast_state = armor.found;
        return false;
    }
    else if(Rlast_state == 0 && armor.found == 0 && R_lost_count == 0)
    {
        armor.state = ArmorState::LOST;
        Rlast_state = armor.found;
        return false;
    }
    return true;
}

bool ArmorDetector::Rmatcharmor(Armor &armor)
{
    if(!originalimg(armor))
        return false;
    if(!Rfindlight(armor))
    {
        //cout<<"RLIGHT"<<endl;
        return false;
    }
    if(!Rfindarmor(armor))
    {
        //cout<<"RARMOR"<<endl;
        return false;
    }
    if(!PnPget(armor))
    {
        //cout<<"RPNP"<<endl;
        return false;
    }
    return true;
}

bool ArmorDetector::PnPget(Armor &armor)
{
    armor.pnp.clear();
    if(armor.l[0].x <= 0 || armor.l[3].x <= 0 || armor.r[1].x <= 0 || armor.r[2].x <= 0)
        return false;
    armor.pnp.push_back(armor.l[1]);//左上
    armor.pnp.push_back(armor.r[2]);//右上
    armor.pnp.push_back(armor.r[3]);//右下
    armor.pnp.push_back(armor.l[0]);//左下
//   cout<<armor.l[1]<<" "<<armor.r[2]<<" "<<armor.r[3]<<" "<<armor.l[0]<<endl;

    return true;
}

bool ArmorDetector::isCoupleLight(RotatedRect &llightcontour, RotatedRect &rlightcontour)
{
    //cout<<"左："<<llightcontour.angle<<"  右："<<rlightcontour.angle<<endl;
    float Contour_angle = abs(llightcontour.angle - rlightcontour.angle);   //角度差
    /*筛选角度差过大的灯条如/ \和\ / */
    if (Contour_angle >= 12.0)
    {
        if(debug.debug_armor_error_print)
        {
            cout<<"灯条角度差过大:"<<Contour_angle<<endl;
        }
        return false;
    }
    //长度差比率
    float Contour_Len1 = abs(llightcontour.size.height - rlightcontour.size.height) /
                         max(llightcontour.size.height, rlightcontour.size.height);
    //宽度差比率
    float Contour_Len2 = abs(llightcontour.size.width - rlightcontour.size.width) /
                         max(llightcontour.size.width, rlightcontour.size.width);
    //筛选形状大小相差太大的
    //cout<<Contour_Len1<<" "<<Contour_Len2<<endl;
    if (Contour_Len1 > 0.35 && Contour_Len2 > 0.44)
    {
        if(debug.debug_armor_error_print)
        {
            cout<<"灯条大小差异过大:"<<Contour_Len1<<" "<<Contour_Len2<<endl;
        }
        return false;
    }
    return true;
}

bool ArmorDetector::ArmorBox(Armor &armor, size_t&i, size_t&j)
{
    if( j - i > 1)
    {
        float x_limit = armor.lightcontour[j].center.x - armor.lightcontour[i].center.x;
        for(size_t m = i; m < j; m++)
        {
            float x_gap = armor.lightcontour[m].center.x - armor.lightcontour[i].center.x;
            if(x_gap > x_limit)
            {
                if(debug.debug_armor_error_print)
                {
                    cout<<"该组中间存在灯条"<<endl;
                }
                 return false;
            }
        }
    }
    Point center = (armor.lightcontour[i].center + armor.lightcontour[j].center) / 2.0;
    float width, height, angle;
    width = sqrt(pow(armor.lightcontour[j].center.x - armor.lightcontour[i].center.x, 2) +
                 pow(armor.lightcontour[j].center.y - armor.lightcontour[i].center.y, 2));
    height = max(armor.lightcontour[i].size.height, armor.lightcontour[j].size.height)*2.2;
    angle = atan2(armor.lightcontour[j].center.y - armor.lightcontour[i].center.y,
                  armor.lightcontour[j].center.x - armor.lightcontour[i].center.x) * 180 / CV_PI;
    if(abs(angle) > 12)
    {
        if(debug.debug_armor_error_print)
            cout<<"装甲板角度错误"<<angle<<endl;
        return false;
    }
    //cout<<"装甲板角度"<<angle<<endl;
    armor.box = Rect(center - Point(width / 2.0, height / 2.0), Size(width, height));
    armor.rect = RotatedRect(center, Size(width, height), angle);
    float area = armor.rect.size.area();
    if(area < 200 || area > 240000)
    {
        if(debug.debug_armor_error_print)
            cout<<"装甲板面积错误"<<area<<endl;
        return false;
    }
    //cout<<"装甲板面积"<<area<<endl;
    float ratio = armor.rect.size.width / armor.rect.size.height;
    //通过长宽比例计算装甲板类型
    //cout<<"装甲板长宽比例:"<<ratio<<endl;
    if(ratio >= 1.65 && ratio <= 2.90)
        armor.armortype = BIG;
    else if(ratio >= 1.0 && ratio <= 1.45)
        armor.armortype = SMALL;
    else if(ratio > 2.90 || ratio < 1.0 ||(ratio > 1.45 && ratio < 1.65))
    {
        if(debug.debug_armor_error_print)
            cout<<"装甲板长宽比例错误:"<<ratio<<endl;
        return false;
    }
    //cout<<ratio<<endl;
    //判断窗户等过亮区域
    armor.ROI = armor.armor(armor.box & Rect(Point(0, 0), armor.armor.size()));
    cv::cvtColor(armor.ROI, armor.ROI, COLOR_BGR2GRAY);
    if(mean(armor.ROI)[0] >= 75.0)
    {
        if(debug.debug_armor_error_print)
            cout<<"过亮区域"<<mean(armor.ROI)[0]<<endl;
        return false;
    }
    armor.found = 1;
    return true;
}

RotatedRect &ArmorDetector::adjustlight(RotatedRect& rec, const int mode)
{
    using std::swap;
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;
    //宽度大于高度
    if (mode == WIDTH_GREATER_THAN_HEIGHT)
    {
        if (width < height)
        {
            swap(width, height);
            angle += 90.0;
        }
    }
    while (angle >= 90.0) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;
    //角度向上
    if (mode == ANGLE_TO_UP)
    {
        if (angle >= 45.0)
        {
            swap(width, height);
            angle -= 90.0;
        }
        else if (angle < -45.0)
        {
            swap(width, height);
            angle += 90.0;
        }
    }
return rec;
}

void ArmorDetector::adjustarmor(Rect &box, Mat &src)
{
    box.y -= box.height / 2.0 * 1.1;
    box.height *= 2.0 * 1.1;
    box &= Rect(cv::Point(0, 0), src.size()); 
}

void ArmorDetector::Gamma(Mat &src, Mat &dst, float fgamma)
{
    CV_Assert(src.data);
    unsigned char lut[256];
    for(int i = 0; i < 256; i++)
    {
        lut[i] = saturate_cast<uchar>(pow(i / 255.0, fgamma) * 255.0f);
    }
    src.copyTo(dst);
    MatIterator_<Vec3b> it, end;
    for(it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; ++it)
    {
        (*it)[0] = lut[(*it)[0]];
        (*it)[1] = lut[(*it)[1]];
        (*it)[2] = lut[(*it)[2]];
    }
}

bool ArmorDetector::getArmorNum(Armor &armor)
{
    cv::Mat temp;
    for(auto &box : armor.boxes)
    {
        //adjustarmor(box, armor.img);
        armor.NUM = armor.img(cv::Rect(box));
        Gamma(armor.NUM, armor.NUM, 0.8);
        resize(armor.NUM, armor.NUM, Size(28, 28), INTER_LINEAR);
        cvtColor(armor.NUM, armor.NUM, COLOR_BGR2GRAY);
        armor.id = classifier(armor.NUM);
        // switch(armor.id)
        // {
        //     case 0:
        //         break;
        //     case 1:
        //         break;
        //     case 2:
        //         break;
        //     case 3:
        //         break;
        //     case 4:
        //         break;
        //     case 5:
        //         break;
        //     case 6:
        //         break;
        //     case 7:
        //         break;
        //     case 8:
        //         break;
        //     case 9:
        //         break;
        // }
    }
    return true;
}

bool ArmorDetector::LRun(Armor &armor)
{
    if(!Lmatcharmor(armor))
    {
        //cout<<"左相机未识别到装甲板"<<endl;
        armor.found = 0;
        armor.fire = 0;
        armor.ypd = Lypd;
    }
    LStateDetection(armor);
    //cout<<"程序识别状态："<<to_string(Llast_state)<<" 左识别状态："<<armor.state<<" 装甲板识别状态："<<to_string(armor.found)<<"识别次数："<<L_lost_count<<endl;
    // if(armor.state == ArmorState::LOST)
    // {
    //     return false;
    // }
    // else if(armor.state == ArmorState::FLOST)
    // {
    //     armor.ypd = Lypd;
    //     return false;
    // }
    if((armor.state == ArmorState::FLOST || armor.state == ArmorState::LOST) && spin_mode == 0)
    {
        armor.fire = 0;
        armor.ypd = Lypd;
    }
    else if((armor.state == ArmorState::SHOOT || armor.state == ArmorState::FIRST) && spin_mode == 0)
    {
        //ofstream file("../configure/ldata.txt", ios::app);
        solve.Lsolve(armor.pnp, armor.armortype);
        armor.ypd = solve.lypd;
        Lypd = armor.ypd;
        if(armor.ypd.x > -5.0 && armor.ypd.y < 5.0)
            armor.fire = 0;
        else
            armor.fire = 0;
        // if (file.is_open()) 
        // {
        //     file <<cv::getTickCount()/cv::getTickFrequency()<<":"<< Lypd.x <<" "<< Lypd.y <<" "<< Lypd.z << endl;
        //     file.close();
        // //std::cout << "Float value appended to file successfully." << std::endl;
        // }
        // if(solve.Lsolve(armor.pnp, armor.armortype))
        // {
        //     armor.ypd = solve.lypd;
        //     Lypd = armor.ypd;
        //     if(armor.ypd.x > -6.0 && armor.ypd.x < 6.0)
        //         armor.fire = 1;
        //     else
        //         armor.fire = 0;
        // }
        // else
        // {
        //     armor.ypd = Lypd;
        //     if(armor.ypd.x > -6.0 && armor.ypd.x < 6.0)
        //         armor.fire = 1;
        //     else
        //         armor.fire = 0;
        // }
    }
    //bool s = Lspindetector.run(armor);
    //cout<<"小陀螺模式："<<s<<endl;
    // if(armor.state == ArmorState::FIRST)
    // {
    //     solve.Lsolve(armor.pnp, armor.armortype);
    //     predict.LinitState(solve.lpoint_3D);
    //     armor.ypd = predict.Lpredict(armor, solve.lpoint_3D);
    //     Llost_xyz = predict.lxyz;
    //     if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //         armor.fire = 1;
    //     else
    //         armor.fire = 0;
    //     return true;
    // }
    // else if(armor.state == ArmorState::SHOOT)
    // {
    //     solve.Lsolve(armor.pnp, armor.armortype);
    //     armor.ypd = predict.Lpredict(armor, solve.lpoint_3D);
    //     Llost_xyz = predict.lxyz;
    //     cout<<"LO:"<<solve.lypd<<endl<<"LK:"<<armor.ypd<<endl;
    //     if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //         armor.fire = 1;
    //     else
    //         armor.fire = 0;
    //     return true;
    // }
    // else if(armor.state ==ArmorState::FLOST)
    // {
    //     predict.LinitState(Llost_xyz, 0);
    //     armor.ypd = predict.Lpredict(armor, Llost_xyz);
    //     Llost_xyz = predict.lxyz;
    //     //cout<<"L掉帧情况："<<armor.state<<"此时预测值："<<armor.ypd<<endl;
    //     if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //         armor.fire = 1;
    //     else
    //         armor.fire = 0;
    //     return true;
    // }
    //Lypd = armor.ypd;
    //Lspindetector.run(armor);
    //cout<<"小陀螺模式: "<<armor.spin_mode<<endl;
    // if(armor.spin_mode)
    // {
    //     armor.ypd.x += Lspindetector.spin_yaw*0.01;
    // }
    //cout<<"LO:"<<solve.lypd<<endl<<"LK:"<<armor.ypd<<endl;
    return true;
}

bool ArmorDetector::RRun(Armor &armor)
{
    if(!Rmatcharmor(armor))
    {
        armor.found = 0;
        armor.fire = 0;
        armor.ypd = Rypd;

    }
    RStateDetection(armor);
    // if(armor.state == ArmorState::LOST)
    // {
    //     return false;
    // }
    // else if(armor.state == ArmorState::FLOST)
    // {
    //     armor.ypd = Rypd;
    //     return false;
    // }
    if (armor.state == ArmorState::FLOST || armor.state == ArmorState::LOST)
    {
        return false;
    }
    else if(armor.state == ArmorState::SHOOT || armor.state == ArmorState::FIRST)
    {
        solve.Rsolve(armor.pnp, armor.armortype);
        armor.ypd = solve.rypd;
        Rypd = armor.ypd;
        if(armor.ypd.x > -5.0 && armor.ypd.y < 5.0)
            armor.fire = 1;
        else
            armor.fire = 0;
        // if(solve.Rsolve(armor.pnp, armor.armortype))
        // {
        //     armor.ypd = solve.rypd;
        //     Rypd = armor.ypd;
        //     if(armor.ypd.x > -6.0 && armor.ypd.x < 6.0)
        //         armor.fire = 1;
        //     else
        //         armor.fire = 0;
        // }
        // else
        // {
        //     armor.ypd = Rypd;
        //     if(armor.ypd.x > -6.0 && armor.ypd.x < 6.0)
        //         armor.fire = 1;
        //     else
        //         armor.fire = 0;
        // }
        // armor.ypd = solve.rypd;
        // Rypd = armor.ypd;
        // if(armor.ypd.x > -6.0 && armor.ypd.x < 6.0)
        //     armor.fire = 1;
        // else
        //     armor.fire = 0;
    }
    // if(armor.state == ArmorState::FIRST)
    // {
    //     solve.Rsolve(armor.pnp, armor.armortype);
    //     predict.RinitState(solve.rpoint_3D);
    //     armor.ypd = predict.Rpredict(armor, solve.rpoint_3D);
    //     Rlost_xyz = predict.rxyz;
    //     if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //         armor.fire = 1;
    //     elseif(solve.Rsolve(armor.pnp, armor.armortype))
        // {
        //     armor.ypd = solve.rypd;
        //     Rypd = armor.ypd;
        //     if(armor.ypd.x > -6.0 && armor.ypd.x < 6.0)
        //         armor.fire = 1;
        //     else
        //         armor.fire = 0;
        // }
        // else
        // {
        //     armor.ypd = Rypd;
        //     if(armor.ypd.x > -6.0 && armor.ypd.x < 6.0)
        //         armor.fire = 1;
        //     else
        //         armor.fire = 0;
        // }
    //         armor.fire = 0;
    //     return true;
    // }
    // else if(armor.state == ArmorState::SHOOT)
    // {
    //     solve.Rsolve(armor.pnp, armor.armortype);
    //     armor.ypd = predict.Rpredict(armor, solve.rpoint_3D);
    //     Rlost_xyz = predict.rxyz;
    //     cout<<"RO:"<<solve.rypd<<endl<<"RK:"<<armor.ypd<<endl;
    //     if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //         armor.fire = 1;
    //     else
    //         armor.fire = 0;
    //     return true;
    // }
    // else if(armor.state ==ArmorState::FLOST)
    // {
    //     predict.RinitState(Rlost_xyz);
    //     armor.ypd = predict.Rpredict(armor, Rlost_xyz);
    //     Rlost_xyz = predict.rxyz;
    //     //cout<<"R掉帧情况："<<armor.state<<"此时预测值："<<armor.ypd<<endl;
    //     if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //         armor.fire = 1;
    //     else
    //         armor.fire = 0;
    //     return true;
    // }
    //cout<<"程序识别状态："<<to_string(Rlast_state)<<" 右识别状态："<<armor.state<<" 装甲板识别状态："<<to_string(armor.found)<<"识别次数："<<R_lost_count<<endl;
    // if(armor.state == ArmorState::LOST || armor.state == ArmorState::FLOST)
    // {
    //     armor.ypd = Lypd;
    //     return false;
    // }
    // solve.Rsolve(armor.pnp, armor.armortype);
    // //armor.ypd = solve.rypd;
    // if(armor.state == ArmorState::FIRST)
    //     Rpredict.initState(solve.rpoint_3D);
    // armor.ypd = Rpredict.predict(solve.rpoint_3D);
    // Rypd = armor.ypd;
    // else if(armor.state == ArmorState::FLOST)
    // {
    //     // Rpredict.initState(Rlost_xyz);
    //     // armor.ypd = Rpredict.predict(Rlost_xyz);
    //     // Rlost_xyz = Rpredict.lxyz;
    //     armor.ypd = Rypd;
    //     if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //         armor.fire = 1;
    //     else
    //         armor.fire = 0;
    //     return false;
    // }
    // if(armor.state == ArmorState::SHOOT || armor.state == ArmorState::FIRST)
    //     solve.Rsolve(armor.pnp, armor.armortype);
    // //armor.ypd = solve.rypd;
    // if(armor.state == ArmorState::FIRST)
    // {
    //     Rpredict.initState(solve.rpoint_3D);
    //     armor.ypd = Rpredict.predict(solve.rpoint_3D);
    //     Rlost_xyz = Rpredict.lxyz;
    // }
    // if (armor.state == ArmorState::SHOOT)
    // {
    //     armor.ypd = Rpredict.predict(solve.rpoint_3D);
    //     Rlost_xyz = Rpredict.lxyz;
    // }
    // if(armor.state == ArmorState::FLOST)
    // {
    //     armor.ypd = Rpredict.predict(Rlost_xyz);
    // }
    // if(armor.ypd.x > -3.0 && armor.ypd.x < 3.0)
    //     armor.fire = 1;
    // else
    //     armor.fire = 0;
    //cout<<"RO:"<<solve.rypd<<endl<<"RK:"<<armor.ypd<<endl;
    return true;
}
