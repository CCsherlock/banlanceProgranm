#include "vision.h"
#include "board.h"
#include "crc.h"
#include "TimeMatch.h"
#include "can.h"

vision_Recv_Struct_t vision_RecvStruct;
vision_Send_Struct_t vision_SendStruct;

DMA_InitTypeDef vision_Rx_DMA_InitStructure;
DMA_InitTypeDef vision_Tx_DMA_InitStructure;
/**
 * @ingroup TDT_DEVICE
 * @defgroup TDT_DEVICE_VISION 视觉通信
 * @note 如果视觉通信不通过，请检查下列  
 * - 串口连接是否正常
 * - 是否已调用初始化以及发送函数
 * - 其他地方是否有定义USART1_IRQHandler函数
 * - 发送接受结构体是否正确
 * - 是否一字节对齐 (#pragma pack(1))
 * @{
 */
///视觉信息，包括帧率和是否离线信息
visionInfo_t visionInfo;
///陀螺仪数据的时间同步
TimeSimultaneity imuTimeMatch(1, sizeof(vec2f), 460800);
/** @} */

u8 tmp_RecvBuff[sizeof(vision_Recv_Struct_t) + 1];
void Vision_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE); //GPIOB，DMA时钟使能
	
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                      //USART1时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9，USART1，TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10，USART1，RX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 460800;
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
    vision_Rx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    vision_Rx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    vision_Rx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff;
    vision_Rx_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    vision_Rx_DMA_InitStructure.DMA_BufferSize = sizeof(tmp_RecvBuff);
    vision_Rx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    vision_Rx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    vision_Rx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    vision_Rx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    vision_Rx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    vision_Rx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    vision_Rx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    vision_Rx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    vision_Rx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    vision_Rx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &vision_Rx_DMA_InitStructure);
    DMA_Cmd(DMA2_Stream5, ENABLE);

    DMA_DeInit(DMA2_Stream7);
    vision_Tx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    vision_Tx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    vision_Tx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&vision_SendStruct;
    vision_Tx_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    vision_Tx_DMA_InitStructure.DMA_BufferSize = sizeof(vision_SendStruct);
    vision_Tx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    vision_Tx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    vision_Tx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    vision_Tx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    vision_Tx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    vision_Tx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    vision_Tx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    vision_Tx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    vision_Tx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    vision_Tx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &vision_Tx_DMA_InitStructure);

    vision_RecvStruct.no_Obj = 1;
}

/**
 * @addtogroup TDT_DEVICE_VISION
 * @brief 视觉数据发送，如果是应答模式( ANSWER_MODE = 1)，则在串口1接收中断发送；否则在陀螺仪计算完之后发送
 * @sa ANSWER_MODE
 */
void vision_Send_Data()
{
    /*****GetValueStart******/

    imuTimeMatch.writeData((char *)vision_SendStruct.gimbalTimeStamp, sizeof(vision_SendStruct.gimbalTimeStamp), 1);
    /***** GetValueEnd *****/
    /*****SetDefaultValue*****/
    vision_SendStruct.frameHeader = 0xA5;
    Append_CRC16_Check_Sum((u8 *)&vision_SendStruct, sizeof(vision_SendStruct));
    //设置传输数据长度
    DMA_Cmd(DMA2_Stream7, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
        ;
    DMA_DeInit(DMA2_Stream7);
    DMA_Init(DMA2_Stream7, &vision_Tx_DMA_InitStructure);
    //打开DMA,开始发送
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

//串口中断
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
        if (tmp_RecvBuff[0] == 0xA5 && Verify_CRC16_Check_Sum((u8 *)&tmp_RecvBuff, sizeof(vision_RecvStruct)))
        {

            visionInfo.offlineFlag = 0;
            visionInfo.visionCnt++;
            tmp = vision_RecvStruct.no_Obj;
            memcpy((u8 *)(&vision_RecvStruct), tmp_RecvBuff, sizeof(vision_RecvStruct));
            /*应答模式*/
            imuTimeMatch.timeFixed(vision_RecvStruct.recvTime, sizeof(vision_RecvStruct), sizeof(vision_SendStruct), recvTime);
#if ANSWER_MODE
            vision_Send_Data();
#endif
        }
        while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE)
            ;
        DMA_DeInit(DMA2_Stream5);
        DMA_Init(DMA2_Stream5, &vision_Rx_DMA_InitStructure);
        DMA_SetCurrDataCounter(DMA2_Stream5, sizeof(tmp_RecvBuff));
        DMA_Cmd(DMA2_Stream5, ENABLE);
    }
}
