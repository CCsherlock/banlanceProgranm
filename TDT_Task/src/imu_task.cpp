/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-22 21:41:57
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-07-02 16:59:24
 * @FilePath: \Projectd:\TDT2023\Programe\TDT-Frame\TDT_Task\src\imu_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

ImuCalc *mpu6050Cal;
//ImuCalc *bmi;

eulerAngle angleForWatch;
eulerAngle angleForWatchAHRS;
accdata accForWatch1;
gyrodata gyroForWatch1;
float *visionSendYaw, *visionSendPitch;
void startControlTasks(); //等待imu初始化后开启控制任务
/**
  * @brief 陀螺仪任务
  * @note 负责数据读取和解算
  */
void Imu_Task()
{	
	if (mpu6050Cal->forceGetOffset)
	{
		mpu6050Cal->getOffset();
	}
	/*MPU6050读取*/
	uint64_t readImuTime = mpu6050Cal->TDT_IMU_update();
//	mpu6050Out->TDT_IMU_update();
//	if (visionSendYaw != NULL && visionSendPitch != NULL)
//		imuTimeMatch.top(float(readImuTime) / 1e6f) << (vec2f({*visionSendYaw, *visionSendPitch}));

#if !ANSWER_MODE /*应答模式，在有时间同步算法的情况下不使用，时间同步的下位替代*/
//	void vision_Send_Data();
//	vision_Send_Data();
#endif
	angleForWatch = mpu6050Cal->Angle;
	angleForWatchAHRS = mpu6050Cal->AHRS_data.Angle;
	accForWatch1 = mpu6050Cal->acc;
	gyroForWatch1 = mpu6050Cal->gyro;
}

#include "flash_var.h"
void imuInit()
{
	mpu6050Cal = new ImuCalc;
	
	/*当前主控陀螺仪的引脚号*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	mpu6050Cal = new Mpu6050(GPIOC, GPIO_Pin_2, GPIO_Pin_1);

	/*陀螺仪和加速度的方向旋转矩阵*/
	float gyroScaleFactor[3][3] =  {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	mpu6050Cal->setGyroScaleFactor(gyroScaleFactor);
	float accScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	mpu6050Cal->setAccScaleFactor(accScaleFactor);
	
//	bmi = new ImuCalc;
//	bmi = new Bmi088(SPI1,SPI_BaudRatePrescaler_256);
////	float gyroScaleFactorBMI[3][3] =  {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
//		float gyroScaleFactorBMI[3][3] =  {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
//	bmi->setGyroScaleFactor(gyroScaleFactorBMI);
////	float accScaleFactorBMI[3][3] = {{0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
//		float accScaleFactorBMI[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
//	bmi->setAccScaleFactor(accScaleFactorBMI);
//	
//	bmi->init();
//	delayMs(50);
//	bmi->initalAngle();
//	bmi->imu_OK = 1;
	#if FROM_FLASH
	/*从Flash获取校准数据*/
	// IFlash.link(mpu6050Cal->gyro.offset, 2);
	// IFlash.link(mpu6050Cal->acc.offset, 3);
	IFlash.link(mpu6050Cal->KTest, 3);
	IFlash.link(mpu6050Cal->sixCaliFector, 4);
	IFlash.link(mpu6050Cal->sixCaliOffset, 5);
	IFlash.link(mpu6050Cal->gyroCaliOffset,6);
//	vec3f gyroOffset;
//	gyroOffset.data[0] = -50.7874125874126;
//	gyroOffset.data[1] = 29.6986790986791;
//	gyroOffset.data[2] = -0.903574203574204;
//	memcpy(&mpu6050Cal->gyro.offset,&gyroOffset,sizeof(gyroOffset));
	
	
	
	
	#endif
	#if FROM_MANUL
	float accCaliFector[3] = {0.991165494425534,0.996649691628215,0.983589087624697};
	float accCalioffset[3] = {62.921329919064135,25.342098693041486,765.678807961455};
	vec3f gyroOffset;
	gyroOffset.data[0] = -50.7874125874126;
	gyroOffset.data[1] = 29.6986790986791;
	gyroOffset.data[2] = -0.903574203574204;

	memcpy(mpu6050Cal->sixCaliFector,accCaliFector,sizeof(accCaliFector));
	memcpy(mpu6050Cal->sixCaliOffset,accCalioffset,sizeof(accCalioffset));
	memcpy(&mpu6050Cal->gyro.offset,&gyroOffset,sizeof(gyroOffset));
	#endif 
	/*icm20602以及MPU6050初始化*/
	mpu6050Cal->init();
	mpu6050Cal->getOffset();
	delayMs(50);
	mpu6050Cal->initalAngle();
	/*陀螺仪初始化完成标志位*/
	mpu6050Cal->imu_OK = 1;
	
	
	/*icm20602以及MPU6050初始化*/
//	mpu6050Out->init();
//	mpu6050Out->getOffset();
//	delayMs(50);
//	mpu6050Out->initalAngle();
//	/*陀螺仪初始化完成标志位*/
//	mpu6050Out->imu_OK = 1;

	//视觉发送的值的初始化
//	visionSendYaw = &bmi->Angle.yaw;
//	visionSendPitch = &bmi->Angle.pitch;
}

