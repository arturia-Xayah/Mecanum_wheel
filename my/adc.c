#include "adc.h"
My_ADC_Type AD[7];

/*
 * adc��ʼ��
 * ͨ�������뵽adc.h�ļ�
 * Ĭ��7·���
 * */
void MyADC_Init()
{
    int i;
    AD[0].channel=My_AD0;           //һ��ֻ��Ҫ����ͨ�����ã��ֱ��ʲ���Ĭ��
    AD[1].channel=My_AD1;
    AD[2].channel=My_AD2;
    AD[3].channel=My_AD3;
    AD[4].channel=My_AD4;
    AD[5].channel=My_AD5;
    AD[6].channel=My_AD6;
    for(i=0;i<7;i++){
        adc_init(AD[i].channel);
    }
}

/*
 * adc�ɼ�һ�Σ�ȡ5��ֵ��ȥ�������С��ȡƽ��
 * ���زɼ�������ʵֵ
 * nbit��Ĭ��Ϊ10bite����
 * */
static uint16 ADC_Once(int index,ADCRES_enum nbit)
{
    unsigned int sum;
    unsigned int max,min,ad;
    unsigned char i;

    ad=adc_convert(AD[index].channel,nbit);
    sum=ad;
    max=ad;
    min=ad;

    for(i=1;i<5;i++)
    {
        ad=adc_convert(AD[index].channel,nbit);
        max=(ad>max? max:ad);
        min=(ad<min? min:ad);
        sum+=ad;
    }
    sum=sum-max-min;
    return (uint16)(sum/3);
}

/*
 * ��ȡ���е�adc�ɼ�ֵ��ʵ��������ֻ��Ҫ�����������
 * */
int16 ADC_protected=45;
void ADC_Get()
{
    int i;
    for(i=0;i<7;i++)
    {
        AD[i].val=ADC_Once(i,ADC_10BIT);
    }
    AD[0].val=AD[i].val*133.01/1024.0f;
}

#if 0
int16_t Gyro_TurnBuffer[3];
int16_t Gyro_Turn;
int16_t Gyro_TurnLast;
int16_t Gyro_a;
void Get_Turn()
{
    int i;
    get_icm20602_gyro_hardware();
    if(icm_gyro_z>-50 && icm_gyro_z<50){
        icm_gyro_z=0;
    }
    Gyro_TurnBuffer[0]=Gyro_TurnBuffer[1];
    Gyro_TurnBuffer[1]=Gyro_TurnBuffer[2];
    Gyro_TurnBuffer[2]=icm_gyro_z;
    Gyro_TurnLast=Gyro_Turn;
    Gyro_Turn=(Gyro_TurnBuffer[2]*7+Gyro_TurnBuffer[1]*2+Gyro_TurnBuffer[0]*1)*0.1;
    Gyro_a=Gyro_Turn-Gyro_TurnLast;
}

#endif



