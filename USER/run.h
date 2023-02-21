#ifndef RUN_H
#define RUN_H
#include "include.h"


#define X_BUCHANG_LIMIT     1500
#define Z_BUCHANG_LIMIT     3000

#define pvfb 1

extern int16 v_longstr;
extern int16 v_str;
extern int16 v_cir;
extern int16 v_min;
extern int16 v_max;
extern float v_k;
extern int16 err_hang;
extern float direrr;




void CTRL_PwmCompensateCal(uint8 x_or_z);
void Run(void);


























#endif
