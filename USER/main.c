/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//整套推荐IO查看Projecct文件夹下的TXT文本

//打开新的工程或者工程移动了位置务必执行以下操作
//右键单击工程，选择刷新
/*行64 列90 中线为LCenter[] 有效行为finaland */

#include "include.h"

void pwmtest();
uint16 runtime=0;
int main(void)
{
    DisableGlobalIRQ();
    board_init();           //务必保留，本函数用于初始化MPU 时钟 调试串口
    //此处编写用户代码(例如：外设初始化代码等)

    //pwmtest();
    Init_AllData();//初始化所有需要用到的全局变量
    oled_init();
    Key_Init();
//    buzzer_init();
    MoterFTM_Init(15000);
    Enc_Init();
    Steer_Init(SteerTest.minduty,50);
    mt9v03x_init();
    //icm20602_init_spi();//硬件icm
    timer_pit_interrupt_ms(TIMER_1,10);
    oled_fill(0x00);


    EnableGlobalIRQ(0);

    while(1)
    {
        if(mt9v03x_finish_flag)
        {
//            timer_pit_start_us(TIMER_3);
            mt9v03x_finish_flag = 0;
            Pixels_get();//获取二值化图像
            SignalProcess();//图像处理函数
            //midline=0.05*LCenter[5]+ 0.05*LCenter[3]+0.1*LCenter[5]+0.1*LCenter[8]+0.2*LCenter[15]+0.5*LCenter[25];//+(parameterB-0.011)*15
            //menu();
//            if(!Is_Run)
//            {
                menu();
//            }

            /*timer_pit_close(TIMER_3);
            runtime=timer_pit_get_us(TIMER_3);
            myputs(20, 7, "runtime:%6d",runtime);*/
        }

    }
}


void pwmtest()
{
    MoterFTM_Init(15000);
    PWMOut(5000,5000,5000,5000);
    while(1);
}




/*
//四个轮子单独控制
/*void Run()
{
    //速度读取函数在中断timer3里面，详见isr.c

    get_icm20602_gyro_spi();
    if(abs(icm_gyro_z)<50)
    {
        icm_gyro_z=0;
    }

    Servo();
    if(fabs(midline_deviation)<4.0f)
    {
        v_max=v_longstr;
    }else if(finalend<55)
    {
        v_max=v_cir;
    }else
    {
        if(midline_deviation>35)
        {
            v_max=v_cir;
        }else
        {
            v_max=v_longstr-(midline_deviation-4.5)/(35-4.5)*(v_longstr-v_cir);
        }

    }
    if(parameterB>0.75f) parameterB=0.75;
    if(parameterB<-0.75f) parameterB=-0.75;

    Vset=v_max-v_k*(parameterB*parameterB)/0.64*(v_max-v_min);
    DirCal(&pid_dir,45,LCenter[26]+parameterB*2);
    SetSpeed(Vset,0,0);//pid_dir.value*18=80*18
    CLR_PwmCompensate(2);

   CTRL_PwmCompensateCal(2);

    if(Is_Run)
    {
        PID_MotorControl(wheel.v1,wheel.v2,wheel.v3,wheel.v4);
//          PID_MotorCal(&pid_motor_3,wheel.v3,encoder_speed[2]);
//          PWMOut(wheel.v1*50,wheel.v2*50,0,0);
    }else {
        Pid_MotorInit(&pid_motor_1);
        Pid_MotorInit(&pid_motor_2);
        Pid_MotorInit(&pid_motor_3);
        Pid_MotorInit(&pid_motor_4);
        CLOSE_MOTOR;
    }
}*/
