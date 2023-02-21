#include "motor.h"
PWM_TYPE PWMTest_t={0};
Steer_TYPE SteerTest={0};

/*
    舵机初始化，舵机配置结构体赋值
*/
void Steer_Init(uint16 midduty,uint16 freq)
{
    pwm_init(SEVERO1_CHAN,freq,midduty);
}


void MoterFTM_Init(uint32 freq)
{
    gpio_init(Motor_L_DirPin, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(Motor_L,freq,0);

    gpio_init(Motor_R_DirPin, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(Motor_R,freq,0);

    gpio_init(MotorAfter_L_DirPin, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(MotorAfter_L,freq,0);

    gpio_init(MotorAfter_R_DirPin, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(MotorAfter_R,freq,0);

}
void LimitPWM(int32 *p,int32 maxduty,int32 minduty)
{
    if(*p>maxduty)
    {
        *p=maxduty;
    }
    else if(*p<minduty)
    {
        *p=minduty;
    }
}
void PWMOut(int16 left,int16 right,int16 after_left,int16 after_right)
{
    if(left<=0){
        gpio_set(Motor_L_DirPin, 0);
        pwm_duty(Motor_L, -left);
    }
    else{
        gpio_set(Motor_L_DirPin, 1);
        pwm_duty(Motor_L, left);
    }
//-------------------------------------------
    if(right>=0){
        gpio_set(Motor_R_DirPin, 1);
        pwm_duty(Motor_R, right);
    }
    else{
        gpio_set(Motor_R_DirPin, 0);
        pwm_duty(Motor_R, -right);
    }

//-------------------------------------------

    if(after_left<0){
        gpio_set(MotorAfter_L_DirPin, 1);
        pwm_duty(MotorAfter_L, -after_left);
    }
    else{
        gpio_set(MotorAfter_L_DirPin, 0);
        pwm_duty(MotorAfter_L, after_left);
    }

//-------------------------------------------

    if(after_right<=0){
        gpio_set(MotorAfter_R_DirPin, 1);
        pwm_duty(MotorAfter_R, -after_right);
    }
    else{
        gpio_set(MotorAfter_R_DirPin, 0);
        pwm_duty(MotorAfter_R, after_right);
    }
}

void Motor_test()
{
    if(PWMTest_t.Enable){
        PWMOut(PWMTest_t.L,PWMTest_t.R,PWMTest_t.After_L,PWMTest_t.After_R );
    }
}

void Servo()//0为正常1为反转
{
    if(SteerTest.dir>0){
        SteerTest.dir=1;
    }else{
        SteerTest.dir=0;
    }
    if(!SteerTest.dir){
        pwm_duty(SEVERO1_CHAN,SteerTest.minduty);
    }else {
        pwm_duty(SEVERO1_CHAN,SteerTest.maxduty);
    }
}

