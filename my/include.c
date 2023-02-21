#include "include.h"
uint8 Is_Run=0;
int16 act_speed_l,act_speed_r,act_speed_y;
int16 minus_l=0,minus_r=0;    //前后轮差速
int16 x_buchang_l,x_buchang_r;
int16 z_buchang;
int16 Vset;

int16 motor_kp,motor_ki,motor_kd;

void Init_AllData()
{
    v_longstr=320;
    v_str=300;
    v_cir=280;
    v_min=240;
    v_max=0;

    motor_kp=1465;   //速度环1800
    motor_ki=125;//180
    motor_kd=0;

    pid_z.kp=0;    //z轴转向环
    pid_z.ki=0;
    pid_z.kd=0;

    pid_dir.P=53;   //方向环
    pid_dir.D=5;
    pid_dir.I=0;
    pid_dir.p1=43;
    pid_dir.p2=44;
    pid_dir.p3=44.5;
    pid_dir.p4=45;

    SteerTest.maxduty=934;
    SteerTest.minduty=500;
    SteerTest.dir=0;

    wheel.v1=0;
    wheel.v2=0;
    wheel.v3=0;
    wheel.v4=0;

    Pid_ZInit(&pid_z);
//    Pid_MotorInit(&pid_motor_1);
//    Pid_MotorInit(&pid_motor_2);
//    Pid_MotorInit(&pid_motor_3);
//    Pid_MotorInit(&pid_motor_4);
}
