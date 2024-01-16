#include "usart3.h"
#include "crc.h"
#include "lqrCtrl_task.h"
#include "stdio.h"
#include "chassis_task.h"
#include "motion_task.h"
#include "dbus.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
Custom_Recv_Struct_t custom_RecvStruct;
Custom_Send_Struct_t custom_SendStruct;
char customSendData[30];
DMA_InitTypeDef custom_Rx_DMA_InitStructure;
DMA_InitTypeDef custom_Tx_DMA_InitStructure;
u8 tmp_RecvBuff1[sizeof(Custom_Recv_Struct_t) + 1];
u8 u1SendBuf[100];
void Custom_Init_V5(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE); // GPIOB，DMA时钟使能

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // USART3时钟使能

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10，USART3，TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11，USART3，RX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_DeInit(USART3);
    USART_InitStructure.USART_BaudRate = 256000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(DMA1_Stream1);
    custom_Rx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    custom_Rx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
    custom_Rx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff1;
    custom_Rx_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    custom_Rx_DMA_InitStructure.DMA_BufferSize = sizeof(tmp_RecvBuff1);
    custom_Rx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    custom_Rx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    custom_Rx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    custom_Rx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    custom_Rx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    custom_Rx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    custom_Rx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    custom_Rx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    custom_Rx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    custom_Rx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &custom_Rx_DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1, ENABLE);

    DMA_DeInit(DMA1_Stream3);
    custom_Tx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    custom_Tx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
    custom_Tx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&customSendData;
    custom_Tx_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    custom_Tx_DMA_InitStructure.DMA_BufferSize = sizeof(customSendData);
    custom_Tx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    custom_Tx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    custom_Tx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    custom_Tx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    custom_Tx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    custom_Tx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    custom_Tx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    custom_Tx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    custom_Tx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    custom_Tx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &custom_Tx_DMA_InitStructure);
}

void Custom_Init_Cboard(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE); // GPIOB，DMA时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); // GPIOA9，USART1，TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // GPIOB7，USART1，RX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 256000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(DMA2_Stream5);
    custom_Rx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    custom_Rx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    custom_Rx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff1;
    custom_Rx_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    custom_Rx_DMA_InitStructure.DMA_BufferSize = sizeof(tmp_RecvBuff1);
    custom_Rx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    custom_Rx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    custom_Rx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    custom_Rx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    custom_Rx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    custom_Rx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    custom_Rx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    custom_Rx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    custom_Rx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    custom_Rx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &custom_Rx_DMA_InitStructure);
    DMA_Cmd(DMA2_Stream5, ENABLE);

    DMA_DeInit(DMA2_Stream7);
    custom_Tx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    custom_Tx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    custom_Tx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&u1SendBuf;
    custom_Tx_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    custom_Tx_DMA_InitStructure.DMA_BufferSize = sizeof(u1SendBuf);
    custom_Tx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    custom_Tx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    custom_Tx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    custom_Tx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    custom_Tx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    custom_Tx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    custom_Tx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    custom_Tx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    custom_Tx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    custom_Tx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &custom_Tx_DMA_InitStructure);
}

// 串口中断
// void USART3_IRQHandler(void)
//{
//    float recvTime = getSysTimeUs();
//    u8 tmp;
//    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
//    {
//        tmp = USART3->SR;
//        tmp = USART3->DR;
//        DMA_Cmd(DMA1_Stream1, DISABLE);
//        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
//        if (tmp_RecvBuff1[0] == 0xA5 && Verify_CRC16_Check_Sum((u8 *)&tmp_RecvBuff1, sizeof(custom_RecvStruct)))
//        {
//            memcpy((u8 *)(&custom_RecvStruct), tmp_RecvBuff1, sizeof(custom_RecvStruct));
//        }
//        while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE)
//            ;
//        DMA_DeInit(DMA1_Stream1);
//        DMA_Init(DMA1_Stream1, &custom_Rx_DMA_InitStructure);
//        DMA_SetCurrDataCounter(DMA1_Stream1, sizeof(tmp_RecvBuff1));
//        DMA_Cmd(DMA1_Stream1, ENABLE);
//    }
//}

// 串口中断
void USART1_IRQHandler(void)
{
    float recvTime = getSysTimeUs();
    u8 tmp;
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        tmp = USART1->SR;
        tmp = USART1->DR;
        DMA_Cmd(DMA2_Stream5, DISABLE);
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
        if (tmp_RecvBuff1[0] == 0xA5 && Verify_CRC16_Check_Sum((u8 *)&tmp_RecvBuff1, sizeof(custom_RecvStruct)))
        {
            memcpy((u8 *)(&custom_RecvStruct), tmp_RecvBuff1, sizeof(custom_RecvStruct));
        }
        while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE)
            ;
        DMA_DeInit(DMA2_Stream5);
        DMA_Init(DMA2_Stream5, &custom_Rx_DMA_InitStructure);
        DMA_SetCurrDataCounter(DMA2_Stream5, sizeof(tmp_RecvBuff1));
        DMA_Cmd(DMA2_Stream5, ENABLE);
    }
}
extern float legSpeed;
extern float temp[2];
void custom_Send_Data(void)
{
    /*****GetValueStart******/
    custom_SendStruct.fi = balance.fiFb;
    custom_SendStruct.theta[LEFT] = balance.angleFb[LEFT];
    custom_SendStruct.theta[RIGHT] = balance.angleFb[RIGHT];
    custom_SendStruct.XSpeed = balance.speedFb;
    custom_SendStruct.angleSpeed[LEFT] = balance.angleSpeedFb[LEFT];
    custom_SendStruct.angleSpeed[RIGHT] = balance.angleSpeedFb[RIGHT];
    custom_SendStruct.fiSpeed = balance.fiSpeedFb;
    custom_SendStruct.setLegTorque[LEFT] = balance.legTorque[LEFT];
    custom_SendStruct.motorSpeed[LEFT] = balance.chassis->legSpeed[LEFT];
    custom_SendStruct.setTheta[LEFT] = balance.angleSet[LEFT];
    custom_SendStruct.setChaTorque[LEFT] = balance.legTorque[LEFT];
    custom_SendStruct.setChaTorque[RIGHT] = balance.legTorque[RIGHT];
    /***** GetValueEnd *****/
    //    sendData(2,
    //             legMotor[LEFT]->megSpeed,
    //						 legMotor[LEFT]->megSpeed_encode);

    //    sendData(5,
    //             custom_SendStruct.XSpeed[LEFT],
    //						 custom_SendStruct.theta[LEFT],
    //						 custom_SendStruct.angleSpeed[LEFT],
    //						 custom_SendStruct.fi,
    //						 custom_SendStruct.fiSpeed);

    //    sendData(3,
    //             custom_SendStruct.fi,
    //						 balance.chassis->legAngel[LEFT],
    //						 custom_SendStruct.theta[LEFT]);

    //    sendData(3,
    //             custom_SendStruct.fiSpeed,
    //						 balance.chassis->legSpeed[LEFT],
    //						 custom_SendStruct.angleSpeed[LEFT]);

    //    sendData(3,
    //             custom_SendStruct.fi,
    //						 custom_SendStruct.theta[LEFT],
    //						 custom_SendStruct.setTorque[LEFT]);

    //    sendData(2,
    //							temp[0],temp[1]);

//        sendData(3,
//    							balance.angleSpeedFb[LEFT],balance.speedFb,balance.fiSpeedFb);

        sendData(6,
    							balance.angleFb[LEFT],balance.angleSet[LEFT],balance.legTorque[LEFT],balance.angleFb[RIGHT],balance.angleSet[RIGHT],balance.legTorque[RIGHT]);

//    sendData(2,
//             custom_SendStruct.setChaTorque[LEFT], custom_SendStruct.setChaTorque[RIGHT]);

//        sendData(3,
//    							custom_SendStruct.fi,custom_SendStruct.theta[LEFT],custom_SendStruct.theta[RIGHT]);
}

void sendData(int cnt, ...)
{
    va_list args;
    va_start(args, cnt);
    int allLength = 0;
    for (uint8_t i = 0; i < cnt; i++)
    {
        if (i == 0)
            allLength += sprintf((char *)(u1SendBuf + allLength), "%.3f", va_arg(args, double));
        else
            allLength += sprintf((char *)(u1SendBuf + allLength), ",%.3f", va_arg(args, double));
    }

    allLength += sprintf((char *)(u1SendBuf + allLength), "\r\n");
    va_end(args);
    /*****SetDefaultValue*****/
#if defined USE_MAIN_CTRL_RM_CBOARD
    // 设置传输数据长度
    DMA_Cmd(DMA2_Stream7, DISABLE);
    custom_Tx_DMA_InitStructure.DMA_BufferSize = allLength;
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
        ;
    DMA_DeInit(DMA2_Stream7);
    DMA_Init(DMA2_Stream7, &custom_Tx_DMA_InitStructure);
    // 打开DMA,开始发送
    DMA_Cmd(DMA2_Stream7, ENABLE);
#else
    DMA_Cmd(DMA1_Stream3, DISABLE);
    custom_Tx_DMA_InitStructure.DMA_BufferSize = allLength;
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
        ;
    DMA_DeInit(DMA1_Stream3);
    DMA_Init(DMA1_Stream3, &custom_Tx_DMA_InitStructure);
    // 打开DMA,开始发送
    DMA_Cmd(DMA1_Stream3, ENABLE);
#endif
}
