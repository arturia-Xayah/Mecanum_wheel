#include "base.h"
void MyDelay(unsigned int t)//�����ʱ
{
   volatile uint16 i;
   while(t--)
    {
        i=6000;//�����Լ��޸ĵģ�
        while(i--);
    }
}

int16 Limit_Int16(int16 *p,int16 max,int16 min)
{
    if((*p)>max)
    {
        (*p)=max;
    }else
    if((*p)<min)
    {
        (*p)=min;
    }
    return *p;
}
