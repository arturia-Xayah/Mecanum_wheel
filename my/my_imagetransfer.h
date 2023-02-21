
#ifndef _MY_IMAGETRANSFER_H
#define _MY_IMAGETRANSFER_H

#include "headfile.h"

//#define gray
#define bin
#define BAUD 460800
#define send_max (57600)  //1s发送最大字节
#define coefficient_num 20  //参数个数
extern float my_car_data[coefficient_num];
/******************************************************
 *                                  图像传输协议
 *  传输头校验：0x01 0x02 0x03 0x04 0x05  （5字节）
 *  图像：200*70字节 直接传输    共14000字节
 *  运行参数：50个 每位4字节 一倍校验 共400字节
 *  拟图：70（行）*7（组）       共490字节
 ******************************************************/

struct Byte8_Struct
{
 uint8_t bit1:1;
 uint8_t bit2:1;
 uint8_t bit3:1;
 uint8_t bit4:1;
 uint8_t bit5:1;
 uint8_t bit6:1;
 uint8_t bit7:1;
 uint8_t bit8:1;
};


typedef struct //传输结构体
{
    uint8_t Verific[5];//校验位
    uint8_t Pixels_width;//图像宽度 186
    uint8_t Pixels_height;//图像高度 70
    uint8_t Coefficient_Num;//datanum 50
    uint8_t Parameter_Num;  //parnum 1
    uint8_t Data_Con[coefficient_num*8];//数据 data_give
    //uint32_t Par_Con[10];//不用
    uint8_t Pixels[675];//压缩图像
}My_Data_Type;

//传输大小计算
#define Verific_Len 9
#define Pixels_Len  675
#define Data_Len    (coefficient_num*8)
#define Par_Len    40
//传输总长度
#define Send_Data_Len   (Verific_Len+Pixels_Len+Data_Len)//14895
//接口函数
void  My_Data_Uart_Init(void);//传输初始化
void My_LiveTransfer(void);//实时传输


#endif



