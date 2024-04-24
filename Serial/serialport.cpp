/*
 * @Author: BRMBC
 * @Date: 2024-01-24 18:10:57
 * @LastEditors: BRMBC
 * @LastEditTime: 2024-04-24 13:46:01
 * @FilePath: /RMvision/Serial/serialport.cpp
 * @Description: 串口通信配置
 * 本文件仅用于个人使用，不允许商业，有问题请联系作者(QQ:3069619801)
 */
#include "serialport.h"
#include "tools.h"

#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <errno.h>      // ERROR Number Definitions
#include <termios.h>    // POSIX Terminal Control Definitions
#include <memory.h>

extern Color ENEMYCOLOR;
extern int targetNum;

using namespace cv;
using namespace std;

SerialPort usart("/dev/ttyUSB0", 0); //串口名称需修改     115200波特率

SerialPort::SerialPort()
{

}

SerialPort::SerialPort(const char* filename, int buadrate)
{
    file_name_ = filename;
    buadrate_  = buadrate;
//    serial_mode = NO_INIT;

    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
    // No terminal will control the process
    last_fd = fd;
    if(fd == -1)
    {
        printf("open_port wait to open %s .\n", file_name_);
//        NOTICE("wait serial " << file_name_,1);
        return;
    }
    else if(fd != -1)
    {
        fcntl(fd, F_SETFL,0);
//        NOTICE("port is open " << file_name_,1);
        printf("port is open %s.\n", file_name_);
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate_ == 1)
    {
        cfsetispeed(&port_settings, B921600);       // set baud rates

        cfsetospeed(&port_settings, B921600);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port

}

// pc -> stm32
void SerialPort::send_data(void)
{
    unsigned char buff[100]={0};
    memcpy(buff,&usart_tx,sizeof(usart_tx_t));
    if(sizeof(usart_tx_t) != write(fd, buff, sizeof(usart_tx_t)))
    {
        cout << "!!! send data failure !!!" << fd << endl;
        restart_serial();
        cout << "restart fd" << fd << endl;
    }
}

void SerialPort::ready_send(char lfound, float lnum, char lfire, float lyaw, float lpitch, float ldistance, char rfound, int rnum, char rfire, float ryaw, float rpitch, float rdistance)
{
    this->usart_tx.lfound = lfound;
    this->usart_tx.lnum = lnum;
    this->usart_tx.lfire = lfire;
    this->usart_tx.lyaw = lyaw;
    this->usart_tx.lpitch = lpitch;
    this->usart_tx.ldistance = ldistance;
    this->usart_tx.rfound = rfound;
    this->usart_tx.rnum = rnum;
    this->usart_tx.rfire = rfire;
    this->usart_tx.ryaw = ryaw;
    this->usart_tx.rpitch = rpitch;
    this->usart_tx.rdistance = rdistance;
}

// stm32 -> pc
int SerialPort::read_data(unsigned char *read_buffer, int size)
{

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer */

    int read_size = read(fd, read_buffer, size); /* Read the data     */  
    if(read_size == -1 || read_size == 0)
    {
        restart_serial();
        return 0;
    }

    return read_size;
}

void SerialPort::restart_serial(void)
{
//    cout << "test restart !!" << fd << " " << last_fd << endl;
    close(fd);
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC);// Read/Write access to serial port
//    cout << serial_mode << endl;
    if(fd == -1 && last_fd != -1)
    {
        printf("open_port wait to open %s .\n", file_name_);
//        NOTICE("wait serial",1);
        last_fd = fd;
        return;
    }
    else if(fd != -1 && last_fd == -1)
    {
        fcntl(fd, F_SETFL,0);
//        NOTICE("port is open",1);
        printf("port is open %s.\n", file_name_);
        last_fd = fd;
    }else
    {
        last_fd = fd;
        return;
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate_==0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate_ == 1)
    {
        cfsetispeed(&port_settings, B921600);       // set baud rates
        cfsetospeed(&port_settings, B921600);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port

}

void serial_transmit_data::get_xy_data(int16_t x, int16_t y, int16_t yaw, int16_t pitch, int found ,int state)
{
    size = 12;
    raw_data[0] = 0xaa;

    raw_data[1]  = x & 0xff;
    raw_data[2]  = (x>>8) &0xff;
    raw_data[3]  = y & 0xff;
    raw_data[4]  = (y>>8) &0xff;

    raw_data[5]  = pitch & 0xff;
    raw_data[6]  = (pitch>>8) &0xff;
    raw_data[7]  = yaw & 0xff;
    raw_data[8]  = (yaw>>8) &0xff;
    raw_data[9]  = static_cast<unsigned char>(found);
    raw_data[10] = static_cast<unsigned char>(state);
    raw_data[11] = 0xbb;
}
