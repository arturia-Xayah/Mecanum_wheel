#include "key.h"

#ifdef ZF_KEY
#define KEY_NUM 3
PIN_enum PinThree[3];
PIN_enum *Pin_Ptr=PinThree;

/*
 * �������������İ�����ʼ��������������ɵ����װ���
* */
void Key_Init()
{
    Pin_Ptr[0]=B2;       //�ܽ����ã��ڴ˸��İ���
    Pin_Ptr[1]=B4;
    Pin_Ptr[2]=B5;
    gpio_init(Pin_Ptr[0],GPI,0,GPIO_INT_CONFIG);
    gpio_init(Pin_Ptr[1],GPI,0,GPIO_INT_CONFIG);
    gpio_init(Pin_Ptr[2],GPI,0,GPIO_INT_CONFIG);
}

#else
#define KEY_NUM 7
PIN_enum Pin[7];
PIN_enum *Pin_Ptr=Pin;
/*
 * ������ʼ��������
 * ��Ҫ���Ĺܽ����ã�ֱ�ӵ��ü���
 * */
void Key_Init()
{
    uint8_t i;
    Pin_Ptr[Menu_up]=C21;       //�ܽ����ã��ڴ˸��İ���
    Pin_Ptr[Menu_down]=C19;
    Pin_Ptr[Menu_left]=C17;
    Pin_Ptr[Menu_right]=C22;
    Pin_Ptr[Menu_center]=C20;
    Pin_Ptr[Menu_flash]=C16;
    Pin_Ptr[Menu_page]=C18;

    for(i=0;i<7;i++)
    {
        gpio_init(Pin_Ptr[i],GPI,0,GPIO_PIN_CONFIG);
    }
}

#endif

/*
 *����ɨ�躯����û���������̣�
 *����ɨ�躯����û���������̣���һֱ���£���᷵��ֵ���ٶȻ��������������
 *�����ٶȿ���ͨ������count_Limit������
 *nΪ��Ҫɨ��İ�����Ŀ
 *����ֵΪ Key_DirTypeö�����͵����ݣ�ʵ���Ͼ���0-n��
 * */
uint8_t count_Limit=20;
uint8_t Key_Down()
{
    uint8_t i,receive_value;
    uint16_t count=0;
    for(i=0;i<KEY_NUM;i++)
    {
        receive_value=gpio_get(Pin_Ptr[i]);
        if(!receive_value)
        {
            MyDelay(8);
            break;
        }
    }
    if(i>=KEY_NUM)
    {
        count_Limit=25;
        return KEY_NUM;
    }
    if(!gpio_get(Pin_Ptr[i]))
    {
        while(gpio_get(Pin_Ptr[i])==0)
        {
            MyDelay(5);
            count++;
            if(count>=count_Limit)
            {
                count_Limit--;
                if(count_Limit<10)
                {
                    count_Limit=10;
                }
                break;
            }
        }
    }
    return i;
}


/*
 * ���а��������ļ��������ڰ������ͣ��
 * ����������£��򷵻�1�����򷵻�0,
 * nΪɨ��İ�����Ŀ
*/
/*uint8_t Key_Stop()
{
    uint8_t i;
    uint8_t flag=0;
    for(i=0;i<KEY_NUM;i++)
    {
        if(gpio_get(Pin_Ptr[i])==0)
        {
            //MyDelay(10);
            if(gpio_get(Pin_Ptr[i])==0)
            {
                flag=1;
            }
        }
    }
    return flag;
}*/







