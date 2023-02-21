#ifndef ENCODER_H
#define ENCODER_H

#include "SEEKFREE_ABSOLUTE_ENCODER.h"
#include "stdlib.h"
//相关定义可以查看"SEEKFREE_ABSOLUTE_ENCODER.h"文件
#define FORWARD_L     C9
#define FORWARD_R     C8
#define AFTER_L        B12
#define AFTER_R        A8


#define WHEEL_C         192             //车轮周长192mm
#define ENCODER_PULSE   512            //编码器转动一周脉冲数
#define WHEEL_PULSE    (512*104/50)     //车轮转动一周脉冲数（104车轮齿轮数，50编码器齿轮数）


extern int16 encoder_speed[4];  //调用ReadSpeed后读取的数据放在这儿

void Enc_Init(void);
void ReadSpeed(void);
//char Encoder_Wrong(int16 limit);
//void ENCODE_SpeedCm(int16* act_speed1, int16* act_speed2, int16* act_speed3, int16* act_speed4);
#endif


