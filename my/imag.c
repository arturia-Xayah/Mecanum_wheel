#include "imag.h"
uint8 Ostu_Threshold=0;   //ͼ����ֵ
uint8 Threshold_Max=130;  //ͼ����ֵ����
uint8 Threshold_Min=100;
uint8 midline;
byte L_black[ROW];
byte R_black[ROW];
byte LCenter[ROW];
byte LoseL[ROW];    //ÿ��ͼ���������Ҷ���  1���� 0δ��
byte LoseR[ROW];
byte finalend=ROW;  //Ϊÿ��ͼ����Ч����
byte Mid = (Max_Col + Min_Col) >> 1;    //ÿ֡ͼ���0���м�ֵ


/**********��򷨣����Э�����********************/
#define GrayScale 256
#define pixelSum (COL * ROW)
uint8 otsuTh()
{
    uint16 histogram[GrayScale] = { 0 };
    uint32 iMulHistogram[GrayScale] = { 0 };
    uint16 i;
    uint8 MinValue, MaxValue;
    uint16 w_back = 0, w_front = 0;
    uint32 u_back_sum = 0, u_front_sum = 0;
    float u_back = 0, u_front = 0;
    float deltaTmp = 0, deltaMax = 0;
    uint8 threshold = 0;
    uint32 sum = 0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    unsigned char* ptr = &mt9v03x_image[0][0];
    for ( i = 0; i < ROW * COL; i++)
        histogram[ptr[i]]++; //������ֵ��Ϊ����������±�
    //��ȡ�ܵ�����ֵ
    for (i = 0; i < GrayScale - 1; i++)
    {
        iMulHistogram[i] = i * histogram[i];
        sum += iMulHistogram[i];
    }

    for (MinValue = 0; MinValue < GrayScale - 1 && histogram[MinValue] == 0; MinValue++)
        ; //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = GrayScale - 1; MaxValue > MinValue && histogram[MaxValue] == 0; MaxValue--)
        ; //��ȡ���Ҷȵ�ֵ
    //�����Ҷȼ�[0,255]
    if (MaxValue == MinValue)
        return (uint8)MaxValue; // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return (uint8)MinValue; // ͼ����ֻ�ж�����ɫ
    //back���������ص㣻front��ǰ�����ص�
    //w��������u��ƽ��ֵ��

    for (i = MinValue; i < MaxValue; i++) // i��Ϊ��ֵ
    {
        //����ǰ���������ص����
        w_back += histogram[i];
        w_front = pixelSum - w_back;
        //����ǰ�����������ܻҶ�ֵ
        u_back_sum += iMulHistogram[i];
        u_front_sum = sum - u_back_sum;
        //����ǰ����������ƽ���Ҷ�ֵ
        u_back = u_back_sum / (float)w_back;
        u_front = u_front_sum / (float)w_front;
        //������䷽��
        deltaTmp = w_back * w_front * (u_back - u_front) * (u_back - u_front);
        //�������䷽��
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8)i;
        }
    }
    return threshold;
}
/**********ͼ���ֵ��********************/
void Pixels_get()
{
    uint16_t i;
    Ostu_Threshold=otsuTh();
    if(Ostu_Threshold<Threshold_Min)Ostu_Threshold=Threshold_Min;
    if(Ostu_Threshold>Threshold_Max)Ostu_Threshold=Threshold_Max;
    for(i=0;i<ROW*COL;i++)
        *(p_Pixels[0]+i)=(*(mt9v03x_image[0]+i)>Ostu_Threshold)*255;
}

/**********oled��ʾͼ��********************/
void DisplayImage_WithOLED()
{
    byte j;
    for(j=0;j<ROW;j++)
    {
        p_Pixels[j][LCenter[j]]=Black;
        p_Pixels[j][L_black[j]]=Black;
        p_Pixels[j][R_black[j]]=Black;
    }
    oled_dis_bmp(ROW,COL,p_Pixels[0],Ostu_Threshold);
}
