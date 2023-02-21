#ifndef ENCODER_H
#define ENCODER_H

#include "SEEKFREE_ABSOLUTE_ENCODER.h"
#include "stdlib.h"
//��ض�����Բ鿴"SEEKFREE_ABSOLUTE_ENCODER.h"�ļ�
#define FORWARD_L     C9
#define FORWARD_R     C8
#define AFTER_L        B12
#define AFTER_R        A8


#define WHEEL_C         192             //�����ܳ�192mm
#define ENCODER_PULSE   512            //������ת��һ��������
#define WHEEL_PULSE    (512*104/50)     //����ת��һ����������104���ֳ�������50��������������


extern int16 encoder_speed[4];  //����ReadSpeed���ȡ�����ݷ������

void Enc_Init(void);
void ReadSpeed(void);
//char Encoder_Wrong(int16 limit);
//void ENCODE_SpeedCm(int16* act_speed1, int16* act_speed2, int16* act_speed3, int16* act_speed4);
#endif


