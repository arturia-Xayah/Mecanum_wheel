#ifndef IMAG_H
#define IMAG_H
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_OLED.h"
#define byte int8_t
#define Max_Col COL-2   //防止数组越界，最小列和最大列
#define Min_Col 2
#define White   255
#define Black   0

extern byte L_black[ROW];
extern byte R_black[ROW];
extern byte LCenter[ROW];
extern byte LoseL[ROW];
extern byte LoseR[ROW];
extern byte finalend;
extern byte Mid;
extern uint8 Ostu_Threshold;
extern uint8 Threshold_Max;
extern uint8 Threshold_Min;
extern uint8 midline;
void Pixels_get(void);//二值化函数
void DisplayImage_WithOLED(void);//图像显示函数

#endif
