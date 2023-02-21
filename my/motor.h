#ifndef motorR_H
#define motorR_H
#include "zf_pwm.h"
#include "zf_gpio.h"
#define SERVO_MOTOR_DUTY(x)         (10000/(1000/50)*(1+x/180))
#define SEVERO1_CHAN        PWM2_CH1_A0//���pwm���ͨ��

#define New_PWM4
#ifdef New_PWM4


#define MotorAfter_R                 PWM4_CH3_B8 //pwm���ͨͨ��
#define MotorAfter_R_DirPin      B10             //��������ķ���1:��ת��0:��ת
#define MotorAfter_L                 PWM4_CH4_B9
#define MotorAfter_L_DirPin      B11
#define Motor_L            PWM4_CH1_B6
#define Motor_L_DirPin A15
#define Motor_R            PWM4_CH2_B7
#define Motor_R_DirPin B3

#endif

#define     CLOSE_MOTOR     PWMOut(0,0,0,0)
/*
 * ���pwm���ռ�ձȽṹ��
 * */
typedef struct{
    uint8 Enable;
    int16 L;
    int16 R;
    int16 After_L;
    int16 After_R;
}PWM_TYPE;
extern PWM_TYPE PWMTest_t;

typedef struct{
    int16 maxduty;
    int16 minduty;
    int16 out;
    int16 dir;
}Steer_TYPE;
extern Steer_TYPE SteerTest;

void Steer_Init(uint16 midduty,uint16 freq);
void MoterFTM_Init(uint32 freq);
void LimitPWM(int32 *p,int32 maxduty,int32 minduty);
void PWMOut(int16 left,int16 right,int16 after_left,int16 after_right);
void Motor_test();
void Servo();
#endif

