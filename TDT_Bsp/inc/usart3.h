#ifndef _USART3_H_
#define _USART3_H_
#include "board.h"
/**
 * @addtogroup TDT_DEVICE_CUSTOM
 * @{
 */

#pragma pack(1)
///视觉发送结构体（MCU->NUC）.
struct Custom_Send_Struct_t
{
	uint8_t frameHeader;		  ///<0xA5
	/*↓↓↓↓↓↓↓↓↓↓↓custom data start↓↓↓↓↓↓↓↓↓↓↓*/
	uint8_t lqrKLoad = 0;
	/*↑↑↑↑↑↑↑↑↑↑↑ custom data end ↑↑↑↑↑↑↑↑↑↑↑*/
	uint16_t CRC16CheckSum;
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
void Custom_Init(void);
void custom_Send_Data(void);
#ifdef __cplusplus
extern "C"
{
#endif

	void USART3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif