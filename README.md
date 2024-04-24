<!--
 * @Author: BRMBC
 * @Date: 2024-04-24 13:46:28
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 16:52:07
 * @FilePath: /RMvision/README.md
 * @Description: 
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
-->
# RMvision 说明文档

## 项目介绍
  本项目为成都工业学院—涉科赛思战队的哨兵机器人自瞄程序，有问题可咨询负责人`摸鱼王(QQ:3069619801)`

## 项目环境
  - C++11
  - CMake 3.5+
  - OpenCV 4.5.0
  - Eigen 3.3.9+
  - MindVision-Linux-SDK

## 项目结构
```

├── Armor
│   ├── armorBox.h
│   ├── armorDetector.cpp
│   ├── armorDetector.h
│   ├── armorClassifier.cpp
│   ├── armorClassifier.h
│   ├── spinDetector.cpp
│   └── spinDetector.h  
├── build
├── camera_device
│   ├── include
│   │   ├── CameraApi.h
│   │   ├── CameraDefine.h
│   │   ├── CameraStatus.h
│   │   └── mindcamera.h
│   ├── lib
│   └── src
│       └── mindcamera.cpp
├── configure
├── extools
│   ├── predictor.cpp
│   ├── predictor.h
│   ├── solver.cpp
│   ├── solver.h
│   ├── tools.cpp
│   └── tools.h
├── serial
│   ├── serialport.cpp
│   └── serialport.h
├── wthread
│   ├── wthread.cpp
│   └── wthread.h
├── CMakeLists.txt
├── main.cpp
└── README.md

```
## 编译运行

编译
```bash
mkdir build
cd build
cmake ..
make
```
打开串口
```bash
sudo chmod 777 /dev/ttyUSB0
```

运行
```bash
./RMvision
```

## 项目框架

**Armor**: 装甲板识别模块，包括装甲板参数、装甲板检测、装甲板分类、自瞄功能。
**camera_device**: 摄像头驱动模块，包括摄像头API、二次开发文件。
**extools**: 额外工具模块，包括卡尔曼滤波预测器、PNP求解器、其他工具函数。
**serial**: 串口通信模块，包括串口通信API。
**wthread**: 线程管理模块，包括线程管理API。

## 重要模块说明

*armorDetecor*是装甲板的主识别模块， *armorClassifier*是装甲板的分类模块， *spinDetector*是小陀螺检测功能模块。

### armorclassifier
其中对于装甲板的数字识别分类，我使用的Eigen实现的CNN网络。其输入图像预处理过程为: 

1. 选取灯条内装甲板ROI区域并进行适当扩大，为灰度图
2. resize为（28，28）
3. gamma矫正处理，增亮

最后输出为0~10的数字，其所代表含义如下：

```
{"base", "hero", "engineer", "infantry3", "infantry4", "infantry5", "sentry", "outpost", "error101", "error010", "error111"};
```

### predictor

卡尔曼滤波预测器，用于预测下一帧的位置和姿态。


## 存在问题

1. 装甲板分类器参考西安电子科技大学IRobot战队的装甲板分类模型，分类效果可以，但我使用会拖慢程序进程，查看CPU占用率低，资源利用低，有待优化
2. 卡尔曼滤波预测存在问题，数据跳动变化大，导致预测结果不准确，有待优化
3. 未实现小陀螺判断算法，有待实现
4. 线程管理模块存在问题，按每帧从串口线程中获取数据会导致整个程序停滞，有待优化