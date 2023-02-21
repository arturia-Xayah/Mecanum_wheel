#include "encoder.h"

void Enc_Init()
{
    encoder_init_spi(FORWARD_L);
    encoder_init_spi(FORWARD_R);
    encoder_init_spi(AFTER_L);
    encoder_init_spi(AFTER_R);
    timer_pit_interrupt_ms(TIMER_3,10);
}

/*
 * 角度编码器读取四个编码器的速度脉冲值放在encoder_speed[]数组中，
 * 读取完毕后将标志位 Is_SpeedGet 置一，
 * 记得用完了数据将标志位清零
 * 清零可以采用宏定义 Clr_Encoder_SpeedFlag !!!
 * */
int16 encoder_speed[4]={0};
char Is_SpeedGet=0;
void ReadSpeed()
{
    int16 encoder_buffer[4];

    encoder_buffer[0]=-1*encoder1_speed_spi(FORWARD_L);
    encoder_buffer[1]=encoder2_speed_spi(FORWARD_R);
    encoder_buffer[2]=-1*encoder3_speed_spi(AFTER_L);
    encoder_buffer[3]=encoder4_speed_spi(AFTER_R);
    encoder_speed[0] = (int16)(encoder_buffer[0]*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
    encoder_speed[1] = (int16)(encoder_buffer[1]*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
    encoder_speed[2] = (int16)(encoder_buffer[2]*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
    encoder_speed[3] = (int16)(encoder_buffer[3]*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
}

/*
 * 速度保护函数，防止程序跑飞，速度过大
 * limit速度正常上限
 * 如果都超过上限，返回1表示异常，此时应该调用停止电机程序
 * 返回0表示正常
 * */
/*char Encoder_Wrong(int16 limit)
{
    if(abs(encoder_speed[0])>limit
        &&abs(encoder_speed[1])>limit
        &&abs(encoder_speed[2])>limit
        &&abs(encoder_speed[3])>limit)
    {
        return 1;

    }else{
        return 0;
    }

}*/
/*

void ENCODE_SpeedCm(int16* act_speed1, int16* act_speed2, int16* act_speed3, int16* act_speed4)
{
    *act_speed1 = (int16)((*act_speed1)*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
    *act_speed2 = (int16)((*act_speed2)*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
    *act_speed3 = (int16)((*act_speed3)*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
    *act_speed4 = (int16)((*act_speed4)*100*1.0f/WHEEL_PULSE*WHEEL_C/10);    //cm/s
}
*/

