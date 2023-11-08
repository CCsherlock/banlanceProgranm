#include "rs485.h"
#include "crc.h"

//双通道高速485，IO切换收发状态，使用硬件切换会因开关管时间造成相应延时

Recv_Struct_t RecvStruct[2]; //两通道接收结构体
Send_Struct_t SendStruct[2]; //两通道发送结构体

uint8_t tmp_RecvBuff[2][sizeof(Recv_Struct_t)+1];//接收缓存

DMA_InitTypeDef rs485_Rx_DMA_InitStructure[2];
DMA_InitTypeDef rs485_Tx_DMA_InitStructure[2];

/**
 * @brief 485串口发送函数
 * @param CHANNEL_X 发送的通道
 */
void RS485_Send_Data(uint8_t CHANNEL_X)
{
    /*****GetValueStart******/
    SendStruct[CHANNEL_X].head.start[0] = 0xFE; //双帧头
    SendStruct[CHANNEL_X].head.start[1] = 0xEE;
    SendStruct[CHANNEL_X].head.reserved = 0x0; //一些用不上参数【据说能唱歌】
    SendStruct[CHANNEL_X].Mdata.ModifyBit = 0xFF;
    SendStruct[CHANNEL_X].Mdata.ReadBit = 0x0;
    SendStruct[CHANNEL_X].Mdata.reserved = 0x0;
    SendStruct[CHANNEL_X].Mdata.Modify = 0;
    SendStruct[CHANNEL_X].Mdata.LowHzMotorCmdIndex = 0;
    SendStruct[CHANNEL_X].Mdata.LowHzMotorCmdByte = 0;
    /***** GetValueEnd *****/
    /*****SetDefaultValue*****/
    SendStruct[CHANNEL_X].CRCdata = crc32_core((uint32_t*)(&SendStruct[CHANNEL_X]), 7); //CRC计算
	
	if(CHANNEL_X == CHANNEL_1)
	{
		CH1_TX_ENABLE;//使能发送
		DMA_Cmd(DMA2_Stream7, DISABLE);
		while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
		DMA_DeInit(DMA2_Stream7);
		DMA_Init(DMA2_Stream7, &rs485_Tx_DMA_InitStructure[CHANNEL_1]);
		DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
		DMA_Cmd(DMA2_Stream7, ENABLE);//打开DMA,开始发送
	}
	else if(CHANNEL_X == CHANNEL_2)
	{
		CH2_TX_ENABLE;//使能发送
		DMA_Cmd(DMA2_Stream6, DISABLE);
		while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE);
		DMA_DeInit(DMA2_Stream6);
		DMA_Init(DMA2_Stream6, &rs485_Tx_DMA_InitStructure[CHANNEL_2]);
		DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
		DMA_Cmd(DMA2_Stream6, ENABLE);//打开DMA,开始发送
	}
}


/**
 * @brief 串口1发送完成中断
 */
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
	{
		while(!USART_GetFlagStatus(USART1,USART_FLAG_TC));//等待发送区完成
		
		CH1_TX_DISABLE;//直接开启会过早失能，需要延时
		
		DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);
	}
}


/**
 * @brief 串口6发送完成中断
 */
void DMA2_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6))
	{
		while(!USART_GetFlagStatus(USART6,USART_FLAG_TC));//等待发送区完成
		
		CH2_TX_DISABLE;//直接开启会过早失能，需要延时
		
		DMA_ClearFlag(DMA2_Stream6,DMA_IT_TCIF6);
		DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);
	}
}

//#include "ut_motor.h"
// #include "chassis_task.h"

/**
 * @brief 串口1接收中断
 */
void USART1_IRQHandler(void)
{
    u8 tmp;
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        tmp = USART1->SR;
        tmp = USART1->DR;
        DMA_Cmd(DMA2_Stream5, DISABLE);
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
        if (tmp_RecvBuff[CHANNEL_1][0] == 0xFE)
        {
            memcpy((u8 *)(&RecvStruct[CHANNEL_1]), tmp_RecvBuff[CHANNEL_1], sizeof(Recv_Struct_t));
			// chassis.leg[CHANNEL_1]->motor[LEFT]->motorRxData_Handle(&RecvStruct[CHANNEL_1]);
			// chassis.leg[CHANNEL_1]->motor[RIGHT]->motorRxData_Handle(&RecvStruct[CHANNEL_1]);
        }
        while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
        DMA_DeInit(DMA2_Stream5);
        DMA_Init(DMA2_Stream5, &rs485_Rx_DMA_InitStructure[CHANNEL_1]);
        DMA_SetCurrDataCounter(DMA2_Stream5, sizeof(tmp_RecvBuff[CHANNEL_1]));
        DMA_Cmd(DMA2_Stream5, ENABLE);
    }
}


/**
 * @brief 串口6接收中断
 */
void USART6_IRQHandler(void)
{
    u8 tmp;
    if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        tmp = USART6->SR;
        tmp = USART6->DR;
        DMA_Cmd(DMA2_Stream1,DISABLE);
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);
        if (tmp_RecvBuff[CHANNEL_2][0] == 0xFE)
        {
           memcpy((u8*)(&RecvStruct[CHANNEL_2]), tmp_RecvBuff[CHANNEL_2], sizeof(Recv_Struct_t));
			// chassis.leg[CHANNEL_2]->motor[LEFT]->motorRxData_Handle(&RecvStruct[CHANNEL_2]);
			// chassis.leg[CHANNEL_2]->motor[RIGHT]->motorRxData_Handle(&RecvStruct[CHANNEL_2]);
        }
        while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
        DMA_DeInit(DMA2_Stream1);
        DMA_Init(DMA2_Stream1, &rs485_Rx_DMA_InitStructure[CHANNEL_2]);
        DMA_SetCurrDataCounter(DMA2_Stream1, sizeof(tmp_RecvBuff[CHANNEL_2]));
        DMA_Cmd(DMA2_Stream1,ENABLE);
    }
}


/**
 * @brief RS485串口初始化
 * @note 对usart，dma以及io进行初始化
 */
void RS485_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_DMA2, ENABLE); //GPIOA，DMA时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_USART6, ENABLE); //USART1时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9，USART1，TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10，USART1，RX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  //GPIOA9，USART1，TX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); //GPIOA10，USART1，RX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//开浮空会有脏东西
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 4800000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

	DMA_Cmd(DMA2_Stream7, DISABLE);//
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
    USART_DMACmd(USART1, USART_DMAReq_Rx|USART_DMAReq_Tx , ENABLE);	  
	DMA_Cmd(DMA2_Stream7, ENABLE);

    USART_DeInit(USART6);
    USART_Init(USART6, &USART_InitStructure);
    USART_Cmd(USART6, ENABLE);

	DMA_Cmd(DMA2_Stream6, DISABLE);//
	DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF7);

    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
    USART_DMACmd(USART6, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(DMA2_Stream6, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_Init(&NVIC_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(DMA2_Stream5);
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_Channel = DMA_Channel_4;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff[CHANNEL_1];
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_DIR = DMA_DIR_PeripheralToMemory;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_BufferSize = sizeof(tmp_RecvBuff[CHANNEL_1]);
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_MemoryInc = DMA_MemoryInc_Enable;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_Mode = DMA_Mode_Normal;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_Priority = DMA_Priority_High;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_FIFOMode = DMA_FIFOMode_Disable;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_MemoryBurst = DMA_MemoryBurst_Single;
    rs485_Rx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &rs485_Rx_DMA_InitStructure[CHANNEL_1]);
    DMA_Cmd(DMA2_Stream5, ENABLE);

    DMA_DeInit(DMA2_Stream7);
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_Channel = DMA_Channel_4;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_Memory0BaseAddr = (uint32_t)&SendStruct[CHANNEL_1];
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_DIR = DMA_DIR_MemoryToPeripheral;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_BufferSize = 34;//sizeof(Send_Struct_t);
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_MemoryInc = DMA_MemoryInc_Enable;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_Mode = DMA_Mode_Normal;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_Priority = DMA_Priority_High;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_FIFOMode = DMA_FIFOMode_Disable;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_MemoryBurst = DMA_MemoryBurst_Single;
    rs485_Tx_DMA_InitStructure[CHANNEL_1].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &rs485_Tx_DMA_InitStructure[CHANNEL_1]);
	
    DMA_DeInit(DMA2_Stream1);
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_Channel = DMA_Channel_5;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff[CHANNEL_2];
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_DIR = DMA_DIR_PeripheralToMemory;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_BufferSize = sizeof(tmp_RecvBuff[CHANNEL_2]);
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_MemoryInc = DMA_MemoryInc_Enable;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_Mode = DMA_Mode_Normal;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_Priority = DMA_Priority_High;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_FIFOMode = DMA_FIFOMode_Disable;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_MemoryBurst = DMA_MemoryBurst_Single;
    rs485_Rx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &rs485_Rx_DMA_InitStructure[CHANNEL_2]);
    DMA_Cmd(DMA2_Stream1, ENABLE);

    DMA_DeInit(DMA2_Stream6);
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_Channel = DMA_Channel_5;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_Memory0BaseAddr = (uint32_t)&SendStruct[CHANNEL_2];
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_DIR = DMA_DIR_MemoryToPeripheral;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_BufferSize = 34;//sizeof(Send_Struct_t);
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_MemoryInc = DMA_MemoryInc_Enable;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_Mode = DMA_Mode_Normal;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_Priority = DMA_Priority_High;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_FIFOMode = DMA_FIFOMode_Disable;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_MemoryBurst = DMA_MemoryBurst_Single;
    rs485_Tx_DMA_InitStructure[CHANNEL_2].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream6, &rs485_Tx_DMA_InitStructure[CHANNEL_2]);
}