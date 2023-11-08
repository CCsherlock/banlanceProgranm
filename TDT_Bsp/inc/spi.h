#ifndef _SPI_H
#define _SPI_H

#include "board.h"

#define CS1_ACCEL_Pin GPIO_Pin_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_GYRO_Pin GPIO_Pin_0
#define CS1_GYRO_GPIO_Port GPIOB

/**
 * @addtogroup TDT_BSP_SPI
 * @{
 */

struct CsPin
{
	GPIO_TypeDef* port;
	uint16_t pin;
};
class Spi
{
	public:
		SPI_TypeDef* spix;//SPIx，用于初始化时操作不用IO
		uint16_t baud;//波特率
		CsPin csPin;
		static bool initFlag[3]; 
		/*SPI构造器*/
		Spi(SPI_TypeDef* spix,uint16_t baud);
		/*SPI初始化*/
		void init();
	
		/*CS脚初始化*/
		void csInit(CsPin csPin);
		/*CS片选操作*/
		virtual void csOn();
		virtual void csOff();
		/*读写一个字节*/
		uint16_t readWriteByte(uint16_t TxData);
		/*读一个字节*/
		u8 readByte(u8 regAddr);
		/*读多个字节*/
		void readBytes(u8 regAddr, u8 len, u8* data);
		/*写一个字节*/
		void writeByte(u8 regAddr, u8 data);
		/*写多个字节*/
		void writeBytes(u8 regAddr, u8 len, u8* data);

};
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);
extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);
extern uint16_t BMI088_read_write_byte(uint16_t txdata);
extern void BMI088_write_single_reg(u8 reg, u8 data);
extern void BMI088_read_muli_reg(u8 reg, u8 *buf, u8 len);
extern void BMI088_read_single_reg(u8 reg, u8 *return_data);
/** @} */

#endif