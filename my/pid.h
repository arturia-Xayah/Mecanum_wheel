#ifndef PID_H
#define PID_H

#include "include.h"

#define BUCHANG_LIMIT   (1500)
#define MECANUM_A   9       //9cm
#define MECANUM_B   9

#define MOTOR_MAX   (8000)
#define MOTOR_MIN   (-8000)

#define DIR_MAX     (7.5)//7.5
#define DIR_MIN     (-7.5)

//定义单个电机调速pid结构体
typedef struct{
    float s_kp;  //直线pid
    float s_ki;
    float s_kd;
    float err;
    float err_1;
    float err_2;
    float inter;
    float value;
    float value_last;
}sPidMotor_Type;
extern sPidMotor_Type pid_motor_1;
extern sPidMotor_Type pid_motor_2;
extern sPidMotor_Type pid_motor_3;
extern sPidMotor_Type pid_motor_4;

//定义一个z轴转向的pid结构体，用于补偿
typedef struct{
    float kp;
    float ki;
    float kd;
    float inter; //积分
    float err;
    float err_1;
    float ec;
    float value;
}sPid_Z;
extern sPid_Z pid_z;

//定义一个用于循线判断转向的pid结构体
typedef struct{
    float P;
    float D;
    float I;
    float p1;
    float p2;
    float p3;
    float p4;
    float err;
    float err_1;
    float ec;
    float inter;
    float value;
    float angle;
}sPID_DIR;
extern sPID_DIR pid_dir;

//定义一个用于存放各个轮子的设定速度的结构体
typedef struct{
    int16 v1,v2,v3,v4;
    int16 vx,vy;
    float vz;
    int16 vl,vr;
}sWheel_Type;
extern sWheel_Type wheel;

//float DirCal(sPID_DIR *pid,uint8 set,uint8 act);//计算偏差赋值给z转角速度计算
float DirCal(sPID_DIR *pid,float err);
float PID_MotorCal(sPidMotor_Type *pid, int16 set, int16 act);//计算每一个电机的输出值
void SetSpeed(int16 vy,int16 vx,float vz);//设置速度值
float PID_ZCal(sPid_Z *pid,float set,float act);//pid打角环
void PID_MotorControl(int set1,int set2, int set3, int set4);//控制输出，包含补偿

void Pid_MotorInit(sPidMotor_Type *pid);
void Pid_ZInit(sPid_Z *pid);

float PID_MotorCal2(sPidMotor_Type *pid, int16 set, int16 act);

#endif
