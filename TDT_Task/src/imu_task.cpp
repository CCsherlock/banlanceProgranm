/******************************
File name: TDT_Task\src\imu_task.cpp
Description: 陀螺仪姿态解算任务
function:
	——————————————————————————————————————————————————————————————————————————
	void Imu_Task(void *pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "imu_task.h"
/**TDT_Device************************/
#include "mpu6050.h"
#include "icm20602.h"
#include "icm20608.h"
#include "scha634_03.h"
#include "TimeMatch.h"
#include "cycle.h"
#include "parameter.h"
#include "flash_var.h"
#include "bmi088.h"

// ImuCalc *mpu6050Cal;
ImuCalc *bmi088Cal;
eulerAngle angleForWatch;
eulerAngle angleForWatchAHRS;
accdata accForWatch1;
gyrodata gyroForWatch1;

float *visionSendYaw, *visionSendPitch;
/**
 * @brief 陀螺仪任务
 * @note 负责数据读取和解算
 */
void Imu_Task()
{
	if (bmi088Cal->forceGetOffset)
	{
		bmi088Cal->getOffset();
	}
	/*MPU6050读取*/
	uint64_t readImuTime = bmi088Cal->TDT_IMU_update();

	angleForWatch = bmi088Cal->Angle;
	angleForWatchAHRS = bmi088Cal->AHRS_data.Angle;
	accForWatch1 = bmi088Cal->acc;
	gyroForWatch1 = bmi088Cal->gyro;
}
/**
右手坐标系
逆时针旋转为正

acc z 下﹢ 上﹣
	x 下﹢ 上﹣
	y 下﹢ 上﹣

gyro z 逆﹢ 顺﹣
	 x 逆﹣ 顺﹢
	 y 逆﹣ 顺﹢
**/
#define C_BOARD_POSSIVE 0
#include "flash_var.h"
void imuInit()
{
	// mpu6050Cal = new ImuCalc;
	bmi088Cal = new ImuCalc;
	/*当前主控陀螺仪的引脚号*/
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	// mpu6050Cal = new Mpu6050(GPIOC, GPIO_Pin_2, GPIO_Pin_1);
	bmi088Cal = new Bmi088(SPI1, SPI_BaudRatePrescaler_256);

	/*陀螺仪和加速度的方向旋转矩阵*/
	//	float gyroScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	// mpu6050Cal->setGyroScaleFactor(gyroScaleFactor);
#if C_BOARD_POSSIVE
	float gyroScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
#else
	float gyroScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
#endif
	bmi088Cal->setGyroScaleFactor(gyroScaleFactor);
	//	float accScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	// mpu6050Cal->setAccScaleFactor(accScaleFactor);
#if C_BOARD_POSSIVE
	float accScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
#else
	float accScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
#endif
	bmi088Cal->setAccScaleFactor(accScaleFactor);
	/*icm20602以及MPU6050初始化*/
	// mpu6050Cal->init();
	// mpu6050Cal->getOffset();
	bmi088Cal->init();
	bmi088Cal->getOffset();
	delayMs(50);
	// mpu6050Cal->initalAngle();
	/*陀螺仪初始化完成标志位*/
	bmi088Cal->imu_OK = 1;

}
