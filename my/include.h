#ifndef INCLUDE_H
#define INCLUDE_H

#include "headfile.h"
//#include "beep.h"
#include "encoder.h"
#include "motor.h"
#include "key.h"
#include "menu.h"
#include "pid.h"
#include "run.h"
#include "fuzzy.h"
#include "findline.h"
typedef enum{
    MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_LEFT_AFTER,
    MOTOR_RIGHT_AFTER,
}MOTOR_DIR_TYPE;




extern int16 act_speed_l,act_speed_r,act_speed_y;
extern int16 minus_l,minus_r;
extern int16 x_buchang_l,x_buchang_r;
extern int16 z_buchang;
extern int16 motor_kp,motor_ki,motor_kd;
extern int16 Vset;















extern uint8 Is_Run;
void Init_AllData();

#endif

