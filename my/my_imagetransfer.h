
#ifndef _MY_IMAGETRANSFER_H
#define _MY_IMAGETRANSFER_H

#include "headfile.h"

//#define gray
#define bin
#define BAUD 460800
#define send_max (57600)  //1s��������ֽ�
#define coefficient_num 20  //��������
extern float my_car_data[coefficient_num];
/******************************************************
 *                                  ͼ����Э��
 *  ����ͷУ�飺0x01 0x02 0x03 0x04 0x05  ��5�ֽڣ�
 *  ͼ��200*70�ֽ� ֱ�Ӵ���    ��14000�ֽ�
 *  ���в�����50�� ÿλ4�ֽ� һ��У�� ��400�ֽ�
 *  ��ͼ��70���У�*7���飩       ��490�ֽ�
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


typedef struct //����ṹ��
{
    uint8_t Verific[5];//У��λ
    uint8_t Pixels_width;//ͼ���� 186
    uint8_t Pixels_height;//ͼ��߶� 70
    uint8_t Coefficient_Num;//datanum 50
    uint8_t Parameter_Num;  //parnum 1
    uint8_t Data_Con[coefficient_num*8];//���� data_give
    //uint32_t Par_Con[10];//����
    uint8_t Pixels[675];//ѹ��ͼ��
}My_Data_Type;

//�����С����
#define Verific_Len 9
#define Pixels_Len  675
#define Data_Len    (coefficient_num*8)
#define Par_Len    40
//�����ܳ���
#define Send_Data_Len   (Verific_Len+Pixels_Len+Data_Len)//14895
//�ӿں���
void  My_Data_Uart_Init(void);//�����ʼ��
void My_LiveTransfer(void);//ʵʱ����


#endif



