#include "run.h"
int16 v_longstr=400;
int16 v_str=375;
int16 v_cir=350;
int16 v_min=240;
int16 v_max=0;



int16 err_hang=0;
float v_k=1;
float direrr=0;
void speedrevise();
int16 AngleErrGet();
void CLR_PwmCompensate(uint8 x_or_z);

float lim=0;
void LimitFloat(float *p,float limit)
{
    (*p) = ((*p)>limit?limit:(*p));
    (*p) = ((*p)<-limit?-limit:(*p));
}
void Run()
{

    direrr=0;
    //速度读取函数在中断timer3里面，详见isr.c



    Servo();
    speedrevise();
    if(midline_deviation>8.0f) midline_deviation=8.0f;

    Vset=v_max-v_k*(midline_deviation*midline_deviation)/(8.0*8.0)*(v_max-v_min);

#define DIRERR_LIM  10//
//    int cnt=0;
//    if(finalend<35)
//    {
//        if(LCenter[15]>45)
//        {
//            direrr=DIRERR_LIM;
//        }else
//        {
//            direrr=-DIRERR_LIM;
//        }
//    }else
//    {
//        for(i=15;i<36;i+=5)
//        {
//            cnt++;
//            direrr+=LCenter[i]-45;
//        }
//        direrr=direrr/cnt;
//        if(direrr>DIRERR_LIM)
//        {
//            direrr=DIRERR_LIM;
//        }
//        if(direrr<-DIRERR_LIM)
//        {
//            direrr=-DIRERR_LIM;
//        }
//    }
//    err_hang=20;
    err_hang=AngleErrGet();
    Limit_Int16(&err_hang, 40, 10);
    if(err_hang>finalend)
    {
        err_hang=finalend-1;
    }

    DirCal(&pid_dir,LCenter[err_hang]-45);//+parameterB*2


    if(!SteerTest.dir)
    {
        if(act_speed_y>350)
        {
            LimitFloat(&pid_dir.value,5.7);
        }else if(act_speed_y>300)
        {
            LimitFloat(&pid_dir.value,6.0);
        }else if(act_speed_y>250)
        {
            LimitFloat(&pid_dir.value,6.1);
        }else
        {
            LimitFloat(&pid_dir.value,6.2);
        }
//        lim=act_speed_y/50.0;
//        if(lim>8.0)   lim=8.0;
//        if(lim<4.0)   lim=4.0;
//        LimitFloat(&pid_dir.value,lim);
        SetSpeed(Vset,0,pid_dir.value);//pid_dir.value*18=10*18
//        SetSpeed(0,0,pid_dir.value);//pid_dir.value*18=10*18
    }else
    {
        SetSpeed(0,Vset,pid_dir.value);//pid_dir.value*18=10*18
    }



    CLR_PwmCompensate(2);

//   CTRL_PwmCompensateCal(1);

    if(Is_Run)
    {
        PID_MotorControl(wheel.v1,wheel.v2,wheel.v3,wheel.v4);
    }else {
        Pid_MotorInit(&pid_motor_1);
        Pid_MotorInit(&pid_motor_2);
        Pid_MotorInit(&pid_motor_3);
        Pid_MotorInit(&pid_motor_4);
        CLOSE_MOTOR;
    }
}



void CTRL_PwmCompensateCal(uint8 x_or_z)
{
    int16 turn=wheel.vr - wheel.vl;
    if (x_or_z == 0)
       {
            x_buchang_l = minus_l * pvfb;
            x_buchang_r = minus_r * pvfb;
            if (x_buchang_l > X_BUCHANG_LIMIT)  x_buchang_l = X_BUCHANG_LIMIT;
            if (x_buchang_l < -X_BUCHANG_LIMIT) x_buchang_l = -X_BUCHANG_LIMIT;
            if (x_buchang_r > X_BUCHANG_LIMIT)  x_buchang_r = X_BUCHANG_LIMIT;
            if (x_buchang_r < -X_BUCHANG_LIMIT) x_buchang_r = -X_BUCHANG_LIMIT;
       }
       else if (x_or_z == 1)
       {
           z_buchang = (int)PID_ZCal(&pid_z, wheel.vr - wheel.vl, act_speed_r - act_speed_l+pid_z.ki*icm_gyro_z*0.0001f);
            if (z_buchang > BUCHANG_LIMIT)      z_buchang = BUCHANG_LIMIT;
            if (z_buchang < -BUCHANG_LIMIT)     z_buchang = -BUCHANG_LIMIT;
       }
       else
       {
            x_buchang_l = minus_l * pvfb;
            x_buchang_r = minus_r * pvfb;
            if (x_buchang_l > X_BUCHANG_LIMIT)  x_buchang_l = X_BUCHANG_LIMIT;
            if (x_buchang_l < -X_BUCHANG_LIMIT) x_buchang_l = -X_BUCHANG_LIMIT;
            if (x_buchang_r > X_BUCHANG_LIMIT)  x_buchang_r = X_BUCHANG_LIMIT;
            if (x_buchang_r < -X_BUCHANG_LIMIT) x_buchang_r = -X_BUCHANG_LIMIT;

//            Limit_Int16(&turn,150,-150);
            z_buchang = (int)PID_ZCal(&pid_z, turn, act_speed_r - act_speed_l+pid_z.ki*icm_gyro_z*0.0001f);
            if (z_buchang > BUCHANG_LIMIT)      z_buchang = BUCHANG_LIMIT;
            if (z_buchang < -BUCHANG_LIMIT)     z_buchang = -BUCHANG_LIMIT;
       }
}

void CLR_PwmCompensate(uint8 x_or_z)
{
    if(x_or_z==0)
    {
        x_buchang_l=0;
        x_buchang_r=0;
    }
    else if (x_or_z==1)
    {
        z_buchang=0;
    }
    else
    {
        x_buchang_l=0;
        x_buchang_r=0;
        z_buchang=0;
    }


}
int16 longstr_cnt=0;

void speedrevise()
{
    if(finalend>45)
    {
        if(midline_deviation<0.8&&midline_deviation1<3.5f&&midline_deviation2<8.0f)//长直道
        {
//            longstr_cnt++;
//            v_max=v_str+longstr_cnt*10;
//            if(v_max>v_longstr)
//            {
//                v_max=v_longstr;
//                longstr_cnt-=1;
//            }
            v_max=v_longstr;
        }
        else if(midline_deviation<0.85&&midline_deviation1<3)//短直道
        {
            v_max=v_str;
            longstr_cnt=0;
        }else
        {
            v_max=v_cir;
            longstr_cnt=0;
        }
    }
    else
    {
        v_max=v_cir;
        longstr_cnt=0;
    }
}
#define Hang_MAX    35
#define Hang_Min    10

int16 AngleErrGet()
{
    int line;
    if(act_speed_y<200)
    {
        line=Hang_Min;
    }else
    if(act_speed_y<=320)
    {
        if(act_speed_y>300)
        {
            line=(act_speed_y-200)*(Hang_MAX-Hang_Min)/(320-200)+Hang_Min;
        }else
        if(act_speed_y>280)
        {
            line=(act_speed_y-200)*(Hang_MAX-Hang_Min)/(300-200)+Hang_Min;
        }else
        if(act_speed_y>260)
        {
            line=(act_speed_y-200)*(Hang_MAX-Hang_Min)/(280-200)+Hang_Min;
        }
        else
        if(act_speed_y>240)
        {
            line=(act_speed_y-200)*(Hang_MAX-Hang_Min)/(260-200)+Hang_Min;
        }
        else
        {
            line=(act_speed_y-200)*(Hang_MAX-Hang_Min)/(240-200)+Hang_Min;
        }
    }else
    {
        if(act_speed_y<340)
        {
            line = Hang_MAX + (act_speed_y - 320) / 5;
        }else if(act_speed_y<360)
        {
            line = Hang_MAX + (act_speed_y - 320) / 7;
        }else if(act_speed_y<380)
        {
            line = Hang_MAX + (act_speed_y - 320) / 9;
        }else
        {
            line = Hang_MAX + (act_speed_y - 320) / 11;
        }
    }
    return line;
}
