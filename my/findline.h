#ifndef FINDLINE_H
#define FINDLINE_H
#include"imag.h"
void Data_init(void);
void SignalProcess(void);
extern byte in_sancha;
extern byte in_shizi;
extern float midline_deviation;//���߷���
extern float midline_deviation1;
extern float midline_deviation2;
extern float parameterA;//�ؾ�
extern float parameterB;//б��
extern float k3;
extern byte all_state;
extern void Clear_FlagAll();
extern byte angle_line;//�����
extern byte all_lose;
extern byte left1_flag,left2_flag,right1_flag,right2_flag;
extern byte in_ruku,start_ruku,ruku_count,all_lose_end,line_directe,top_x,top_y;//���
#endif

