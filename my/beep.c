#include "beep.h"


#ifdef BUZZER
void buzzer_delayon(int i)
{
    buzzer_on();
    MyDelay(i);
    buzzer_off();
}

#endif
