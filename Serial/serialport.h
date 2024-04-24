/*
 * @Author: BRMBC
 * @Date: 2024-01-24 18:10:57
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:46:04
 * @FilePath: /RMvision/Serial/serialport.h
 * @Description: 串口通信
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801) 
 */
#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>

struct serial_transmit_data
{
    unsigned char raw_data[20];
    int size;
    unsigned char head = 0xaa;
    unsigned char end = 0xbb;
    void get_xy_data(int16_t x, int16_t y, int16_t yaw, int16_t pitch, int found ,int state);
};

struct serial_receive_data
{
    unsigned char raw_data[10];
    int size;
    unsigned char head = 0xaa;
    unsigned char end = 0xbb;

};

#pragma pack (1)
typedef struct
{
    unsigned char head = 0x0A;      //包头
    char lfound;                    //左云台装甲板识别状态(1:识别 0:未识别)
    int lnum;                       //左云台装甲板识别数字(暂时为用,默认0)
    char lfire;
    float lyaw;                     //左云台yaw轴角度(以相机中心为原点)
    float lpitch;                   //左云台pitch轴角度(以相机中心为原点)
    float ldistance;                //左云台距离(以相机CMOS为零点)
    char rfound;                     //右云台装甲板识别状态(1:识别 0:未识别)
    int rnum;                       //右云台装甲板识别数字(暂时为用,默认0)
    char rfire;
    float ryaw;                     //右云台yaw轴角度(以相机中心为原点)
    float rpitch;                   //右云台pitch轴角度(以相机中心为原点)
    float rdistance;                //右云台距离(以相机CMOS为零点)
    unsigned char end = 0x0D;       //包尾
}usart_tx_t;

typedef  struct
{
    unsigned char head = 0xAB;
    char color;
    float lpitch;
    float rpitch;
    //char  enemy_color;
    unsigned char end =  0xBA;

}usart_rx_t;

#pragma pack ()

class SerialPort
{
public:
    //"/dev/ttyTHS0"  >>  需修改
    SerialPort();
    /**
     * @brief SerialPort
     * @param filename 串口名字
     * @param buadrate 串口波特率,用于stm32通信是0-B115200,用于stm32通信是1-B921600
     */
    SerialPort(const char* filename, int buadrate);

    void restart_serial(void);  // 尝试重连的函数
    void send_data(void);
    int read_data(unsigned char *read_buffer,int size);

    void ready_send(char lfound, float lnum, char lfire, float lyaw, float lpitch, float ldistance, char rfound, int rnum, char rfire, float ryaw, float rpitch, float rdistance);

    int fd;
    int last_fd;

private:

    const char* file_name_;
    int buadrate_;
    float last_bullet_speed;
    usart_tx_t usart_tx;
};
#endif // SERIALPORT_H
