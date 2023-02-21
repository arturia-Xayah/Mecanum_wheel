/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ790875685)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�

//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//�Ҽ��������̣�ѡ��ˢ��
/*��64 ��90 ����ΪLCenter[] ��Ч��Ϊfinaland */

#include "include.h"

void pwmtest();
uint16 runtime=0;
int main(void)
{
    DisableGlobalIRQ();
    board_init();           //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    //�˴���д�û�����(���磺�����ʼ�������)

    //pwmtest();
    Init_AllData();//��ʼ��������Ҫ�õ���ȫ�ֱ���
    oled_init();
    Key_Init();
//    buzzer_init();
    MoterFTM_Init(15000);
    Enc_Init();
    Steer_Init(SteerTest.minduty,50);
    mt9v03x_init();
    //icm20602_init_spi();//Ӳ��icm
    timer_pit_interrupt_ms(TIMER_1,10);
    oled_fill(0x00);


    EnableGlobalIRQ(0);

    while(1)
    {
        if(mt9v03x_finish_flag)
        {
//            timer_pit_start_us(TIMER_3);
            mt9v03x_finish_flag = 0;
            Pixels_get();//��ȡ��ֵ��ͼ��
            SignalProcess();//ͼ������
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
//�ĸ����ӵ�������
/*void Run()
{
    //�ٶȶ�ȡ�������ж�timer3���棬���isr.c

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
