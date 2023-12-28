#ifndef _USART3_H_
#define _USART3_H_
#include "board.h"
/**
 * @addtogroup TDT_DEVICE_CUSTOM
 * @{
 */
#define CUSTOM_PART_SIGN ','
#define CUSTOM_PART_START '/'
#define CUSTOM_PART_END '*'
#pragma pack(1)
///视觉发送结构体（MCU->NUC）.
struct Custom_Send_Struct_t
{
//	uint8_t frameHeader;		  ///<0xA5
	/*↓↓↓↓↓↓↓↓↓↓↓custom data start↓↓↓↓↓↓↓↓↓↓↓*/
	float fi = 0;
	float theta[2] = {0,0};
	float setTheta[2] = {0,0};
	float XSpeed[2] = {0,0};
	float angleSpeed[2] = {0,0};
	float fiSpeed = 0;
	float setTorque[2] = {0,0};
	float motorSpeed[2] = {0,0};
	/*↑↑↑↑↑↑↑↑↑↑↑ custom data end ↑↑↑↑↑↑↑↑↑↑↑*/
//	char frameEnd;
};

///视觉接收结构体（NUC->MCU）.
struct Custom_Recv_Struct_t
{
	uint8_t frameHeader; //0xA5
	/*↓↓↓↓↓↓↓↓↓↓↓custom data start↓↓↓↓↓↓↓↓↓↓↓*/
	uint8_t lqrKChange = 0;
	float lqrK[40];
	/*↑↑↑↑↑↑↑↑↑↑↑ custom data end ↑↑↑↑↑↑↑↑↑↑↑*/
	uint16_t CRC16CheckSum;
};
#pragma pack()


extern Custom_Recv_Struct_t custom_RecvStruct;
extern Custom_Send_Struct_t custom_SendStruct;

///串口初始化
void Custom_Init_V5(void);
void Custom_Init_Cboard(void);
void custom_Send_Data(void);
void sendData(int cnt,...);
#ifdef __cplusplus
extern "C"
{
#endif

void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif