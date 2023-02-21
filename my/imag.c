#include "imag.h"
uint8 Ostu_Threshold=0;   //图像阈值
uint8 Threshold_Max=130;  //图像阈值限制
uint8 Threshold_Min=100;
uint8 midline;
byte L_black[ROW];
byte R_black[ROW];
byte LCenter[ROW];
byte LoseL[ROW];    //每行图像左丢线与右丢线  1丢线 0未丢
byte LoseR[ROW];
byte finalend=ROW;  //为每场图像有效行数
byte Mid = (Max_Col + Min_Col) >> 1;    //每帧图像第0行中间值


/**********大津法（最大协方差法）********************/
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
    //统计灰度级中每个像素在整幅图像中的个数
    unsigned char* ptr = &mt9v03x_image[0][0];
    for ( i = 0; i < ROW * COL; i++)
        histogram[ptr[i]]++; //将像素值作为计数数组的下标
    //获取总的像素值
    for (i = 0; i < GrayScale - 1; i++)
    {
        iMulHistogram[i] = i * histogram[i];
        sum += iMulHistogram[i];
    }

    for (MinValue = 0; MinValue < GrayScale - 1 && histogram[MinValue] == 0; MinValue++)
        ; //获取最小灰度的值
    for (MaxValue = GrayScale - 1; MaxValue > MinValue && histogram[MaxValue] == 0; MaxValue--)
        ; //获取最大灰度的值
    //遍历灰度级[0,255]
    if (MaxValue == MinValue)
        return (uint8)MaxValue; // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return (uint8)MinValue; // 图像中只有二个颜色
    //back：背景像素点；front：前景像素点
    //w：比例；u：平均值；

    for (i = MinValue; i < MaxValue; i++) // i作为阈值
    {
        //更新前景、后景像素点比例
        w_back += histogram[i];
        w_front = pixelSum - w_back;
        //更新前景、后景像素总灰度值
        u_back_sum += iMulHistogram[i];
        u_front_sum = sum - u_back_sum;
        //更新前景、后景像素平均灰度值
        u_back = u_back_sum / (float)w_back;
        u_front = u_front_sum / (float)w_front;
        //计算类间方差
        deltaTmp = w_back * w_front * (u_back - u_front) * (u_back - u_front);
        //求最大类间方差
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8)i;
        }
    }
    return threshold;
}
/**********图像二值化********************/
void Pixels_get()
{
    uint16_t i;
    Ostu_Threshold=otsuTh();
    if(Ostu_Threshold<Threshold_Min)Ostu_Threshold=Threshold_Min;
    if(Ostu_Threshold>Threshold_Max)Ostu_Threshold=Threshold_Max;
    for(i=0;i<ROW*COL;i++)
        *(p_Pixels[0]+i)=(*(mt9v03x_image[0]+i)>Ostu_Threshold)*255;
}

/**********oled显示图像********************/
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
