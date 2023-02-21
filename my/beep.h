#ifndef BEEP_H
#define BEEP_H

#define BUZZER
#ifdef BUZZER

#include "zf_gpio.h"
#include "base.h"
#define BUZZER_PORT D2
#define buzzer_init()   gpio_init(BUZZER_PORT,GPO,0,GPIO_PIN_CONFIG)
#define buzzer_on()     gpio_set(BUZZER_PORT,1)
#define buzzer_off()     gpio_set(BUZZER_PORT,0)

void buzzer_delayon(int i);//蜂鸣器滴一下，i设置延时时间

#endif


#endif
