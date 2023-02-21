#include "my_imagetransfer.h"
#include "ch32v10x_dma.h"
#include "ch32v10x_rcc.h"
float my_car_data[coefficient_num]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
//定义传输结构体
My_Data_Type My_Send_Data;
#define USART_DR_Base 0x40013804

void My_Data_Uart_Init()
{

//    oled_p8x16str(0, 0,"0");
    uart_init(UART_1, BAUD, UART1_TX_A9, UART1_RX_A10);//初始化uart
//
//    DMA_InitTypeDef DMA_InitStructure;
//    DMA_DeInit(DMA1_Channel4);
//
//    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&My_Send_Data;                    //源地址
//    DMA_InitStructure.DMA_MemoryBaseAddr = USART_DR_Base;                        //目标地址
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      //外设作为目标
//    DMA_InitStructure.DMA_BufferSize = Send_Data_Len;                                //传输多少个数据
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不增加
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存地址依次+1
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设每次传输一个字节
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //内存每次传输一个字节
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //非循环模式
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 //优先级最高
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            //非内存到内存模式
//    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
//    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  //使能串口3 DMA功能
//    DMA_Cmd(DMA1_Channel4, ENABLE);
//    oled_p8x16str(0, 0,"1");
}
//将float传输的函数
void My_floattoint32(uint8_t *p_int, uint8_t *p_float)
{
    *p_int = *p_float;
    p_int++;    p_float++;
    *p_int = *p_float;
    p_int++;    p_float++;
    *p_int = *p_float;
    p_int++;    p_float++;
    *p_int = *p_float;
}
void Get_SendData()//填充传输结构体
{
    uint32_t i;
    //传输头检验
    My_Send_Data.Verific[0] = 0x01;
    My_Send_Data.Verific[1] = 0x02;
    My_Send_Data.Verific[2] = 0x03;
    My_Send_Data.Verific[3] = 0x04;
    My_Send_Data.Verific[4] = 0x05;
    My_Send_Data.Pixels_width = 90;
    My_Send_Data.Pixels_height = 60;
    My_Send_Data.Coefficient_Num = coefficient_num;
    My_Send_Data.Parameter_Num = 0;
//    //图像获取

    uint8_t *p_pixels = My_Send_Data.Pixels;//传输变量
    uint8_t *p_pixels_gray = &p_Pixels[0][0];//待压缩变量
    union//定义一个公用体
    {
            uint8_t  Byte;
            struct Byte8_Struct Byte_Bit;
    } BIT;
    for( i = 0;i<675/*Pixels_Len*/;i++)
    {
        //将8位压缩成1位
        BIT.Byte_Bit.bit8=*p_pixels_gray++;
        BIT.Byte_Bit.bit7=*p_pixels_gray++;
        BIT.Byte_Bit.bit6=*p_pixels_gray++;
        BIT.Byte_Bit.bit5=*p_pixels_gray++;
        BIT.Byte_Bit.bit4=*p_pixels_gray++;
        BIT.Byte_Bit.bit3=*p_pixels_gray++;
        BIT.Byte_Bit.bit2=*p_pixels_gray++;
        BIT.Byte_Bit.bit1=*p_pixels_gray++;
        //将这一位传输到里面
        *p_pixels++ = BIT.Byte;
    }
    int32_t *p_data = (int32_t*)My_Send_Data.Data_Con;
        for(uint16_t i = 0;i<coefficient_num;i++)
        {
            My_floattoint32((uint8_t *)p_data, (uint8_t *)&my_car_data[i]);
            *(int32_t *)(p_data + 1) = ~*(int32_t *)p_data;
            p_data += 2;
        }

}
void Send_to_Computer()//通过uart发送传输结构体
{
    uart_putbuff(UART_1,(uint8_t*)&My_Send_Data,Send_Data_Len);
//    while (!DMA_GetFlagStatus(DMA1_IT_TC7));
//    DMA1_Channel4->MADDR=(u32)&My_Send_Data;
//    DMA1_Channel4->CNTR=Send_Data_Len;
//    //
//    while(!USART_GetFlagStatus(UART_1, USART_IT_TC));
//    USART_ClearFlag(UART_1, USART_IT_TC);
//    DMA_ClearFlag(DMA1_IT_TC4);
}
void My_LiveTransfer()//实时传输
{

    //My_Data_Give();//运行参数填充
    Get_SendData();//填充传输结构体
    Send_to_Computer();//通过uart发送传输结构体
}
