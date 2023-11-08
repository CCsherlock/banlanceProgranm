#ifndef _RS485_H_
#define _RS485_H_
#include "board.h"
#define _485USED //使用了485串口，关闭默认视觉串口

#define CHANNEL_1 0 //USART1
#define CHANNEL_2 1	//USART6

#if 0
#define CH1_TX_ENABLE 	GPIOC->BSRRL = GPIO_Pin_4
#define CH1_TX_DISABLE	GPIOC->BSRRH = GPIO_Pin_4
#define CH2_TX_ENABLE	GPIOC->BSRRL = GPIO_Pin_3
#define CH2_TX_DISABLE	GPIOC->BSRRH = GPIO_Pin_3
#else
#define CH2_TX_ENABLE 	GPIOC->BSRRL = GPIO_Pin_4
#define CH2_TX_DISABLE	GPIOC->BSRRH = GPIO_Pin_4
#define CH1_TX_ENABLE	GPIOC->BSRRL = GPIO_Pin_3
#define CH1_TX_DISABLE	GPIOC->BSRRH = GPIO_Pin_3
#endif

// MotorType:
//     A1Go1 - 4.8M baudrate - K_W x1024

typedef int16_t q15_t;

#pragma pack(1)

typedef struct {
	// 定义 数据包头
    unsigned char  start[2];     // 包头
	unsigned char  motorID;      // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
	unsigned char  reserved;
}COMHead;
#pragma pack()

#pragma pack(1)
//发给电机的数据[34字节]
typedef struct {
    COMHead head;    
	struct {
		uint8_t mode; // 关节模式选择
		uint8_t ModifyBit;
		uint8_t ReadBit;
		uint8_t reserved;
		int32_t Modify; 
		q15_t T; // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
		q15_t W; // 期望关节速度 （电机本身的速度）x128, 8 + 7描述	
		int32_t Pos; // 期望关节位置 x 16384/6.2832, 14位编码器
		q15_t K_P; // 关节刚度系数 x2048  4+11 描述
		q15_t K_W; // 关节速度系数 x1024  5+10 描述
		uint8_t LowHzMotorCmdIndex;
		uint8_t LowHzMotorCmdByte;
		uint32_t Res;
	}Mdata;
    uint32_t CRCdata;
}Send_Struct_t;
#pragma pack()

#pragma pack(1)
//电机回传的数据[78字节]
typedef struct {
    COMHead head;
	struct {
		uint8_t mode; 		// 当前关节模式
		uint8_t ReadBit;
		int8_t Temp; 		// 电机当前平均温度   
		uint8_t MError; 	// 电机错误标识
		uint32_t Read;
		int16_t T; 			// 当前实际电机输出力矩		7 + 8 描述
		int16_t W; 			// 当前实际电机速度（高速）	8 + 7 描述
		float LW; 			// 当前实际电机速度（低速）
		int16_t W2;
		float LW2;
		int16_t Acc; 		// 电机转子加速度 15+0 描述  惯量较小
		int16_t OutAcc;
		int32_t Pos; 		// 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
		int32_t Pos2;
		int16_t gyro[3]; 	// 电机驱动板3轴角速度数据
		int16_t acc[3];   	// 电机驱动板3轴加速度数据

		// 力传感器的数据
		int16_t     Fgyro[3];
		int16_t     Facc[3];
		int16_t     Fmag[3];
		uint8_t     Ftemp;
		int16_t     Force16;
		int8_t      Force8;
		uint8_t     FError;
		int8_t      Res[1];
	}Mdata;
    uint32_t    CRCdata;
}Recv_Struct_t;
#pragma pack()

void RS485_Init(void);
void RS485_Send_Data(uint8_t CHANNEL_X);

#ifdef __cplusplus
extern "C"
{
#endif
	void USART1_IRQHandler(void);
	void DMA2_Stream7_IRQHandler(void);
	void USART6_IRQHandler(void);
	void DMA2_Stream6_IRQHandler(void);
#ifdef __cplusplus
}
#endif

extern Recv_Struct_t RecvStruct[2];
extern Send_Struct_t SendStruct[2];
#endif