#ifndef _KEY_H
#define _KEY_H

#include "zf_gpio.h"
#include "common.h"
#include "zf_systick.h"
#include "base.h"

#define ZF_KEY//�˴�������ɵ�������������ʮ�����Ի�

#ifndef ZF_KEY
typedef enum{           //������Ӧ��0-7
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
uint8_t Key_Down();//���������İ���ɨ�����
//uint8_t Key_Stop();//������⣬���·���1�������ֶ�ͣ��
#endif


