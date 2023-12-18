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
extern TimeSimultaneity imuTimeMatch;

// ImuCalc *mpu6050Cal;
ImuCalc *myImuCal;
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
	if (myImuCal->forceGetOffset)
	{
		myImuCal->getOffset();
	}
	/*MPU6050读取*/
	uint64_t readImuTime = myImuCal->TDT_IMU_update();

	angleForWatch = myImuCal->Angle;
	angleForWatchAHRS = myImuCal->AHRS_data.Angle;
	accForWatch1 = myImuCal->acc;
	gyroForWatch1 = myImuCal->gyro;
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
#include "flash_var.h"
void imuInit()
{
	// mpu6050Cal = new ImuCalc;
	myImuCal = new ImuCalc;
	/*当前主控陀螺仪的引脚号*/
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	// mpu6050Cal = new Mpu6050(GPIOC, GPIO_Pin_2, GPIO_Pin_1);
	myImuCal = new Bmi088(SPI1, SPI_BaudRatePrescaler_256);

	/*陀螺仪和加速度的方向旋转矩阵*/
	//	float gyroScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	// mpu6050Cal->setGyroScaleFactor(gyroScaleFactor);
	float gyroScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	myImuCal->setGyroScaleFactor(gyroScaleFactor);
	//	float accScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	// mpu6050Cal->setAccScaleFactor(accScaleFactor);
	float accScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	myImuCal->setAccScaleFactor(accScaleFactor);
	/*icm20602以及MPU6050初始化*/
	// mpu6050Cal->init();
	// mpu6050Cal->getOffset();
	myImuCal->init();
	myImuCal->getOffset();
	delayMs(50);
	// mpu6050Cal->initalAngle();
	/*陀螺仪初始化完成标志位*/
	myImuCal->imu_OK = 1;

	// 视觉发送的值的初始化
	// visionSendYaw = &mpu6050Cal->Angle.yaw;
	// visionSendPitch = &mpu6050Cal->Angle.pitch;
}
