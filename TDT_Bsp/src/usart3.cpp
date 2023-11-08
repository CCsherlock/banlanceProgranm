#include "usart3.h"
#include "crc.h"
#include "lqrCtrl_task.h"
Custom_Recv_Struct_t custom_RecvStruct;
Custom_Send_Struct_t custom_SendStruct;

DMA_InitTypeDef custom_Rx_DMA_InitStructure;
DMA_InitTypeDef custom_Tx_DMA_InitStructure;
u8 tmp_RecvBuff1[sizeof(Custom_Recv_Struct_t) + 1];
void Custom_Init(void)
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
    USART_InitStructure.USART_BaudRate = 115200;
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
    custom_Tx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&custom_SendStruct;
    custom_Tx_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    custom_Tx_DMA_InitStructure.DMA_BufferSize = sizeof(custom_SendStruct);
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
// 串口中断
void USART3_IRQHandler(void)
{
    float recvTime = getSysTimeUs();
    u8 tmp;
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        tmp = USART3->SR;
        tmp = USART3->DR;
        DMA_Cmd(DMA1_Stream1, DISABLE);
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
        if (tmp_RecvBuff1[0] == 0xA5 && Verify_CRC16_Check_Sum((u8 *)&tmp_RecvBuff1, sizeof(custom_RecvStruct)))
        {
            memcpy((u8 *)(&custom_RecvStruct), tmp_RecvBuff1, sizeof(custom_RecvStruct));
        }
        while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE)
            ;
        DMA_DeInit(DMA1_Stream1);
        DMA_Init(DMA1_Stream1, &custom_Rx_DMA_InitStructure);
        DMA_SetCurrDataCounter(DMA1_Stream1, sizeof(tmp_RecvBuff1));
        DMA_Cmd(DMA1_Stream1, ENABLE);
    }
}  
void custom_Send_Data(void)
{
    /*****GetValueStart******/
    custom_SendStruct.lqrKLoad = balance.roboLqr->lqrKLoadFlag;
    /***** GetValueEnd *****/
    /*****SetDefaultValue*****/
    custom_SendStruct.frameHeader = 0xA5;
    Append_CRC16_Check_Sum((u8 *)&custom_SendStruct, sizeof(custom_SendStruct));
    // 设置传输数据长度
    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
        ;
    DMA_DeInit(DMA1_Stream3);
    DMA_Init(DMA1_Stream3, &custom_Tx_DMA_InitStructure);
    // 打开DMA,开始发送
    DMA_Cmd(DMA1_Stream3, ENABLE);
}