#ifndef _ADC_H
#define _ADC_H
#include "zf_adc.h"
#include "SEEKFREE_ICM20602.h"
//-------------------------------------------------------------
//                  电感通道配置
//-------------------------------------------------------------

//    ADC_IN0_A0 = 0,
//    ADC_IN1_A1,
//    ADC_IN2_A2,
//    ADC_IN3_A3,
//    ADC_IN4_A4,
//    ADC_IN5_A5,
//    ADC_IN6_A6,
//    ADC_IN7_A7,
//    ADC_IN8_B0,
//    ADC_IN9_B1,
//    ADC_IN10_C0,
//    ADC_IN11_C1,
//    ADC_IN12_C2,
//    ADC_IN13_C3,
//    ADC_IN14_C4,
//    ADC_IN15_C5,

//电感对应表
#define AD0     ADC_IN1_A1
#define AD1     ADC_IN1_A1//只有adc1
#define AD2     ADC_IN1_A1//只有adc1
#define AD5     ADC_IN1_A1
#define AD6     ADC_IN1_A1
#define AD7     ADC_IN1_A1
#define AD8     ADC_IN1_A1
#define AD9     ADC_IN1_A1
#define AD10    ADC_IN1_A1
#define AD11    ADC_IN1_A1
#define AD12    ADC_IN1_A1
#define AD13    ADC_IN1_A1
#define AD14    ADC_IN1_A1

#define My_AD0  AD0
#define My_AD1  AD14
#define My_AD2  AD7
#define My_AD3  AD1
#define My_AD4  AD9
#define My_AD5  AD5
#define My_AD6  AD6

typedef struct{
    uint16 val;             //adc的值
    ADCCH_enum channel;     //adc的通道
}My_ADC_Type;

void MyADC_Init(void);
void ADC_Get(void);
#endif










