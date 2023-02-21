#include "my_imagetransfer.h"
#include "ch32v10x_dma.h"
#include "ch32v10x_rcc.h"
float my_car_data[coefficient_num]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
//���崫��ṹ��
My_Data_Type My_Send_Data;
#define USART_DR_Base 0x40013804

void My_Data_Uart_Init()
{

//    oled_p8x16str(0, 0,"0");
    uart_init(UART_1, BAUD, UART1_TX_A9, UART1_RX_A10);//��ʼ��uart
//
//    DMA_InitTypeDef DMA_InitStructure;
//    DMA_DeInit(DMA1_Channel4);
//
//    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&My_Send_Data;                    //Դ��ַ
//    DMA_InitStructure.DMA_MemoryBaseAddr = USART_DR_Base;                        //Ŀ���ַ
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      //������ΪĿ��
//    DMA_InitStructure.DMA_BufferSize = Send_Data_Len;                                //������ٸ�����
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ������
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ��ַ����+1
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //����ÿ�δ���һ���ֽ�
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�ڴ�ÿ�δ���һ���ֽ�
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //��ѭ��ģʽ
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                 //���ȼ����
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            //���ڴ浽�ڴ�ģʽ
//    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
//    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  //ʹ�ܴ���3 DMA����
//    DMA_Cmd(DMA1_Channel4, ENABLE);
//    oled_p8x16str(0, 0,"1");
}
//��float����ĺ���
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
void Get_SendData()//��䴫��ṹ��
{
    uint32_t i;
    //����ͷ����
    My_Send_Data.Verific[0] = 0x01;
    My_Send_Data.Verific[1] = 0x02;
    My_Send_Data.Verific[2] = 0x03;
    My_Send_Data.Verific[3] = 0x04;
    My_Send_Data.Verific[4] = 0x05;
    My_Send_Data.Pixels_width = 90;
    My_Send_Data.Pixels_height = 60;
    My_Send_Data.Coefficient_Num = coefficient_num;
    My_Send_Data.Parameter_Num = 0;
//    //ͼ���ȡ

    uint8_t *p_pixels = My_Send_Data.Pixels;//�������
    uint8_t *p_pixels_gray = &p_Pixels[0][0];//��ѹ������
    union//����һ��������
    {
            uint8_t  Byte;
            struct Byte8_Struct Byte_Bit;
    } BIT;
    for( i = 0;i<675/*Pixels_Len*/;i++)
    {
        //��8λѹ����1λ
        BIT.Byte_Bit.bit8=*p_pixels_gray++;
        BIT.Byte_Bit.bit7=*p_pixels_gray++;
        BIT.Byte_Bit.bit6=*p_pixels_gray++;
        BIT.Byte_Bit.bit5=*p_pixels_gray++;
        BIT.Byte_Bit.bit4=*p_pixels_gray++;
        BIT.Byte_Bit.bit3=*p_pixels_gray++;
        BIT.Byte_Bit.bit2=*p_pixels_gray++;
        BIT.Byte_Bit.bit1=*p_pixels_gray++;
        //����һλ���䵽����
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
void Send_to_Computer()//ͨ��uart���ʹ���ṹ��
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
void My_LiveTransfer()//ʵʱ����
{

    //My_Data_Give();//���в������
    Get_SendData();//��䴫��ṹ��
    Send_to_Computer();//ͨ��uart���ʹ���ṹ��
}
