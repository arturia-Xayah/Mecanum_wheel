#ifndef _KEY_H
#define _KEY_H

#include "zf_gpio.h"
#include "common.h"
#include "zf_systick.h"
#include "base.h"

#define ZF_KEY//此处采用逐飞的三独立按键，十分人性化

#ifndef ZF_KEY
typedef enum{           //按键对应表0-7
    Menu_up,
    Menu_down,
    Menu_left,
    Menu_right,
    Menu_center,
    Menu_flash,
    Menu_page,
    No_Key,
}Key_DirType;
typedef struct{
    PIN_enum used_pin;
    Key_DirType key_symble;
}Pin_UseType;
#endif

void Key_Init(void);
uint8_t Key_Down();//不带消抖的按键扫描程序
//uint8_t Key_Stop();//按键检测，按下返回1，用于手动停车
#endif


