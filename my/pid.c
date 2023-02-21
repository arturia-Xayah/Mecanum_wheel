#include "pid.h"

/*
 *
 *      2       1
 *
 *      3       4
 *          轮子的方向
 *
 *
 * */
sWheel_Type wheel;
sPid_Z pid_z;
sPID_DIR pid_dir;

sPidMotor_Type pid_motor_1;
sPidMotor_Type pid_motor_2;
sPidMotor_Type pid_motor_3;
sPidMotor_Type pid_motor_4;


//float DirCal(sPID_DIR *pid,uint8 set,uint8 act)
float DirCal(sPID_DIR *pid,float err)
{
    float p;
    pid->err_1 = pid->err;
    pid->err = err*0.7+pid->err_1*0.3;
    if(pid->err>9.0)
    {
        pid->err=9.0;
    }
    if(pid->err<-9.0)
    {
        pid->err=-9.0;
    }
    pid->ec = pid->err - pid->err_1;

//    if(fabs(pid->err)<0.03&&abs(icm_gyro_z)<150)
//    {
//        pid->inter+=pid->err;
//    }
    if(pid->inter>50)
    {
        pid->inter=50;
    }
    if(pid->inter<-50)
    {
        pid->inter=-50;
    }
//    if(fabs(pid->err)<0.05)
//    {
        p=pid->P;
//    }else if(fabs(pid->err)<0.1)
//    {
//        p=pid->p1;
//    }else if(fabs(pid->err)<0.2)
//    {
//        p=pid->p2;
//    }else if(fabs(pid->err)<0.3)
//    {
//        p=pid->p3;
//    }else
//    {
//        p=pid->p4;
//    }
//    pid->P=Fuzzy_control(pid->err,icm_gyro_z,0);
//    pid->D=Fuzzy_control(pid->err,icm_gyro_z,1);

    pid->value = p*pid->err * 0.01f
                + pid->D*pid->ec * 0.01f;
//                -pid->I*pid->inter*0.01f;

    pid->value = (pid->value > (float)DIR_MAX ? DIR_MAX:pid->value);
    pid->value = (pid->value < (float)DIR_MIN ? DIR_MIN:pid->value);

    return pid->value;
}



#if 1
float PID_MotorCal2(sPidMotor_Type *pid, int16 set, int16 act)
{
    float pid_temp = 0;
    int index=0;
    pid->err_2 = pid->err_1;
    pid->err_1 = pid->err;
    pid->err = set-act;

    if(pid->err>250)
    {
        pid->err=250;
    }
    if(pid->err<-250)
    {
        pid->err=-250;
    }

    pid->s_kp = (float)motor_kp/100.0f;
    pid->s_ki = (float)motor_ki/100.0f;
    pid->s_kd = (float)motor_kd/100.0f;

    if(fabs(pid->err)<230)
    {
        index=1;
    }
    pid->inter = index*pid->s_ki *  pid->err;
    pid_temp = pid->s_kp * (pid->err - pid->err_1)
             + pid->inter
             + pid->s_kd * (pid->err - 2*pid->err_1 + pid->err_2);

    pid->value+=pid_temp;

    if (pid->value > (float)MOTOR_MAX) pid->value = MOTOR_MAX;
    if (pid->value < (float)MOTOR_MIN) pid->value = MOTOR_MIN;

    return pid->value;
}
#endif

float PID_MotorCal(sPidMotor_Type *pid, int16 set, int16 act)
{
    float pid_temp = 0;

        pid->err_1 = pid->err;
        pid->err = set-act;

        pid->s_kp = (float)motor_kp/100.0f;//motor_kp=2000
        pid->s_ki = (float)motor_ki/100.0f;//motor_ki=0//
        pid->s_kd = (float)motor_kd/100.0f;//motor_kd=0

        if(fabs(pid->err)<80.0f)
        {
            pid->inter+=pid->err;
        }
//        if(pid->inter>200.0f)
//        {
//            pid->inter=200.0f;
//        }
//        if(pid->inter<-200.0f)
//        {
//            pid->inter=-200.0f;
//        }
        pid_temp = pid->s_kp * pid->err
                 + pid->s_ki *  pid->inter
                 + pid->s_kd * (pid->err - pid->err_1);

        if (pid_temp > 8000.0f) pid_temp = 8000;
        if (pid_temp < -8000.0f) pid_temp = -8000;

        pid->value = pid_temp;
        return pid->value;
}

void Pid_MotorInit(sPidMotor_Type *pid)
{
    pid->err = 0;
    pid->err_1 = 0;
    pid->err_2 = 0;
    pid->value = 0;
    pid->s_kp=0;
    pid->s_ki=0;
    pid->s_kd=0;
    pid->inter=0;
    pid->value_last=0;
}

void SetSpeed(int16 vy,int16 vx,float vz)//逆时针记为正
{
    wheel.vx=vx;
    wheel.vy=vy;
    wheel.vz=vz;

    wheel.v1 = vy - vx - vz*(MECANUM_A + MECANUM_B);
    wheel.v2 = vy + vx + vz*(MECANUM_A + MECANUM_B);
    wheel.v3 = vy + vx - vz*(MECANUM_A + MECANUM_B);
    wheel.v4 = vy - vx + vz*(MECANUM_A + MECANUM_B);
    wheel.vl = (wheel.v1 + wheel.v3)/2;
    wheel.vr = (wheel.v2 + wheel.v4)/2;


}


float PID_ZCal(sPid_Z *pid,float set,float act)
{
    float pid_temp;
    pid->err_1 = pid->err;
    pid->err = set-act;
    pid->ec = pid->err - pid->err_1;

    pid_temp = pid->kp*pid->err*0.1 + pid->kd*pid->ec*0.1 ;//-pid->ki*icm_gyro_z*0.01f*pid_dir.err

    if (pid_temp > BUCHANG_LIMIT) pid_temp = BUCHANG_LIMIT;
    if (pid_temp < -BUCHANG_LIMIT) pid_temp = -BUCHANG_LIMIT;

    pid->value = pid_temp;
    return pid->value;
}

void Pid_ZInit(sPid_Z *pid)
{
    pid->ec = 0;
    pid->err = 0;
    pid->err_1 = 0;
    pid->inter = 0;
    pid->value = 0;
}

int16 value1,value2,value3,value4;
void PID_MotorControl(int set1,int set2, int set3, int set4)//待修改加减
{
//    PID_MotorCal(&pid_motor_1, set1, encoder_speed[0]);//pid计算
//    PID_MotorCal(&pid_motor_2, set2, encoder_speed[1]);
//    PID_MotorCal(&pid_motor_3, set3, encoder_speed[2]);
//    PID_MotorCal(&pid_motor_4, set4, encoder_speed[3]);

    PID_MotorCal2(&pid_motor_1, set1, encoder_speed[0]);//pid计算
    PID_MotorCal2(&pid_motor_2, set2, encoder_speed[1]);
    PID_MotorCal2(&pid_motor_3, set3, encoder_speed[2]);
    PID_MotorCal2(&pid_motor_4, set4, encoder_speed[3]);


    value1 = (int16)(pid_motor_1.value+0.5-z_buchang+x_buchang_l);
    value2 = (int16)(pid_motor_2.value+0.5+z_buchang+x_buchang_r);
    value3 = (int16)(pid_motor_3.value+0.5-z_buchang-x_buchang_l);
    value4 = (int16)(pid_motor_4.value+0.5+z_buchang-x_buchang_r);

    value1 = (value1>MOTOR_MAX ? MOTOR_MAX:value1);
    value2 = (value2>MOTOR_MAX ? MOTOR_MAX:value2);
    value3 = (value3>MOTOR_MAX ? MOTOR_MAX:value3);
    value4 = (value4>MOTOR_MAX ? MOTOR_MAX:value4);

    value1 = (value1< MOTOR_MIN ? MOTOR_MIN:value1);
    value2 = (value2< MOTOR_MIN ? MOTOR_MIN:value2);
    value3 = (value3< MOTOR_MIN ? MOTOR_MIN:value3);
    value4 = (value4< MOTOR_MIN ? MOTOR_MIN:value4);

    PWMOut(value1,value2,value3,value4);
}
