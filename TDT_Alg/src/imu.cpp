/******************************
File name: TDT_Alg\src\imu.cpp
Description: 陀螺仪姿态解算算法
function:
	——————————————————————————————————————————————————————————————————————————
	void TDT_IMUBotupdate(float half_T, vec3f* gyro, vec3f* acc)
	——————————————————————————————————————————————————————————————————————————
	void TDT_IMUTopupdate(float half_T, vec3f* gyro, vec3f* acc)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19/11.19
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.19 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "imu.h"
#include "my_math.h"
#include "math.h"
#include "TimeMatch.h"
#include "flash_var.h"
#include "iwdg.h"
#include "led_task.h"
#include "usart3.h"
/**
 * @ingroup TDT_ALG
 * @defgroup IMU_ALG 陀螺仪解算算法
 * @brief 该类提供了传统的互补滤波算法，以及大疆开源的AHRS算法库的调用，并且可选择是否使用自动校准、是否使用纯角速度积分的选项，同时作为父类供各种陀螺仪继承
 * @{
 */

using namespace imu;

///@note 二阶互补滤波系数，规律：基本时间常数tau得到基本系数a，Kp=2*a，Ki=a^2;
#define Kp 0.6f ///< proportional gain governs rate of convergence to accelerometer/magnetometer
///@note 二阶互补滤波系数，规律：基本时间常数tau得到基本系数a，Kp=2*a，Ki=a^2;
#define Ki 0.1f ///< integral gain governs rate of convergence of gyroscope biases

#if USE_AHRS_LIB && COMPARE_SELF_LIB
/// J-scope数据测试，互补滤波算法结算出来的角度
eulerAngle angleFromLib;
/// J-scope数据测试，大疆开源库AHRS算法结算出来的角度
eulerAngle angleFromSelf;
#endif

/** @}*/

ImuCalc::ImuCalc() : round({0}), quaternion({1, 0, 0, 0}), err({0}),
					 accValueFector(1), gyroDpsFector(1), gyroScaleFactor(),
					 accScaleFactor()
{
	memset(&Angle, 0, sizeof(Angle));
	memset(&LastAngle, 0, sizeof(LastAngle));
	memset(&nowAngle, 0, sizeof(nowAngle));
	float defaultScaleFactor[3][3] = {DEFAULT_BOARD_INSTALL_SPIN_MATRIX};
	setGyroScaleFactor(defaultScaleFactor);
	setAccScaleFactor(defaultScaleFactor);
}

void ImuCalc::getOffset()
{
	if (!forceGetOffset && IFlash.read() == 0) // 成功读取并且不需要强行矫正
	{
		initalAngle();
		return;
	}

	forceGetOffset = 1;
	boardLed.setError(0, LedES_BlinkFast);
	laser.setError(0, LedES_BlinkFast);

	// 读取失败或需要强行校准时，只校准陀螺仪，不校准加速度，并存入flash
	while (!gyroAutoCalibration())
	{
		if (!forceGetOffset)
		{
			boardLed.setError(0, LedES_BlinkSlow);
			laser.setError(0, LedES_BlinkSlow);
			return;
		}
		gyroAccUpdate();
		cycle.getCycleT();
		delayMs(2);
	}

	boardLed.setError(0, LedES_ConstantLight);
	laser.setError(0, LedES_ConstantLight);
	boardLed.show(1);
	laser.show(1);

	// 看门狗复位时间1.5s
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
	auto oldIWDG_P = IWDG->PR;
	auto oldIWDG_RL = IWDG->RLR;
	IWDG_SetPrescaler(IWDG_Prescaler_64); // 设置IWDG分频系数
	IWDG_SetReload(750);				  // 设置IWDG装载值
	IFlash.save();
	// 恢复看门狗时间
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
	IWDG_SetPrescaler(oldIWDG_P);				  // 设置IWDG分频系数
	IWDG_SetReload(oldIWDG_RL);					  // 设置IWDG装载值
	iwdgFeed();									  // reload
	__set_FAULTMASK(1);							  // 关闭所有中断
	NVIC_SystemReset();							  // 复位
	while (1)
	{
	} // 仅等待复位
}

void ImuCalc::calOffset_AccFromOffset(float accOffset[7])
{
	int i = 0;
	while (fpclassify(accOffset[i]) == FP_NORMAL)
	{
		i++;
	}
	if (i == 6)
	{
		sixCaliFector[0] = (accOffset[0] - accOffset[1]) / (1 / accValueFector * 9.81 * 2);
		sixCaliFector[1] = (accOffset[2] - accOffset[3]) / (1 / accValueFector * 9.81 * 2);
		sixCaliFector[2] = (accOffset[4] - accOffset[5]) / (1 / accValueFector * 9.81 * 2);
		for (u8 i = 0; i < 3; i++)
		{
			if (accOffset[i << 1] > 0)
			{
				sixCaliOffset[i] = accOffset[i << 1] / sixCaliFector[i] - 1 / accValueFector * 9.81;
			}
			else
			{
				sixCaliOffset[i] = accOffset[i << 1] / sixCaliFector[i] + 1 / accValueFector * 9.81;
			}
		}
	}
}

///@warning 需要先获取加速度数据
void ImuCalc::initalAngle()
{
	eulerAngle initial; // 初始欧拉角

	gyroAccUpdate();
	caliSolve();
	ImuCalc::gyroAccUpdate(); // 进行单位换算

#if !USE_AHRS_LIB || (USE_AHRS_LIB && COMPARE_SELF_LIB)
	initial.yaw = 0;
	initial.pitch = atan2(-acc.accValue.data[0], acc.accValue.data[2]) * RAD_TO_ANGLE;
	initial.roll = atan2(acc.accValue.data[1], acc.accValue.data[2]) * RAD_TO_ANGLE;

	// 三角函数运算较为耗时，现行运算存入缓存
	float dataBuf[2][3] = {cosf(initial.yaw / (2 * RAD_TO_ANGLE)), cosf(initial.pitch / (2 * RAD_TO_ANGLE)), cosf(initial.roll / (2 * RAD_TO_ANGLE)),
						   sinf(initial.yaw / (2 * RAD_TO_ANGLE)), sinf(initial.pitch / (2 * RAD_TO_ANGLE)), sinf(initial.roll / (2 * RAD_TO_ANGLE))};

	// 更新四元数
	quaternion.q0 = dataBuf[0][0] * dataBuf[0][1] * dataBuf[0][2] + dataBuf[1][0] * dataBuf[1][1] * dataBuf[1][2];
	quaternion.q1 = dataBuf[0][0] * dataBuf[0][1] * dataBuf[1][2] - dataBuf[1][0] * dataBuf[1][1] * dataBuf[0][2];
	quaternion.q2 = dataBuf[0][0] * dataBuf[1][1] * dataBuf[0][2] + dataBuf[1][0] * dataBuf[0][1] * dataBuf[1][2];
	quaternion.q3 = dataBuf[1][0] * dataBuf[0][1] * dataBuf[0][2] - dataBuf[0][0] * dataBuf[1][1] * dataBuf[1][2];

	// 规范化最新四元数（归一化）
	float norm = sqrt(quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1 +
					  quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
	quaternion.q0 = quaternion.q0 / norm;
	quaternion.q1 = quaternion.q1 / norm;
	quaternion.q2 = quaternion.q2 / norm;
	quaternion.q3 = quaternion.q3 / norm;

	// 直接更新欧拉角
	Angle.roll = atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1, -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * RAD_TO_ANGLE;
	Angle.pitch = asin(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * RAD_TO_ANGLE;

	round.roundYaw = 0;

	nowAngle.pitch = Angle.pitch;
	nowAngle.yaw = Angle.yaw;
	nowAngle.roll = Angle.roll;

#endif
#if USE_AHRS_LIB
	AHRS_init(AHRS_data.ins_quat, acc.accValue.data, AHRS_data.mag);
	float angleLibOut[3] = {0};
	// 航资参考系统获取角度，具体声明参考ahrs_lib.h
	get_angle(AHRS_data.ins_quat, angleLibOut, angleLibOut + 1, angleLibOut + 2);
	// 单位换算
	AHRS_data.nowAngle.roll = angleLibOut[2] * RAD_TO_ANGLE;
	AHRS_data.nowAngle.pitch = angleLibOut[1] * RAD_TO_ANGLE;
	AHRS_data.nowAngle.yaw = angleLibOut[0] * RAD_TO_ANGLE;
#if !COMPARE_SELF_LIB
	nowAngle.roll = angleLibOut[2] * RAD_TO_ANGLE;
	nowAngle.pitch = angleLibOut[1] * RAD_TO_ANGLE;
	nowAngle.yaw = angleLibOut[0] * RAD_TO_ANGLE;
#endif
#endif
}

// void ImuCalc::TDT_accFilter()
// {
// 	u8 i;
// 	int32_t FILT_TMP[ITEMS] = {0};

// 	for (i = FILTER_NUM - 1; i >= 1; i--)
// 	{
// 		FILT_BUF[A_X][i] = FILT_BUF[A_X][i - 1];
// 		FILT_BUF[A_Y][i] = FILT_BUF[A_Y][i - 1];
// 		FILT_BUF[A_Z][i] = FILT_BUF[A_Z][i - 1];
// 	}

// 	FILT_BUF[A_X][0] = acc.calibration.data[x];
// 	FILT_BUF[A_Y][0] = acc.calibration.data[y];
// 	FILT_BUF[A_Z][0] = acc.calibration.data[z];

// 	for (i = 0; i < FILTER_NUM; i++)
// 	{
// 		FILT_TMP[A_X] += FILT_BUF[A_X][i];
// 		FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
// 		FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
// 	}

// 	acc.filter.data[x] = (float)(FILT_TMP[A_X]) / (float)FILTER_NUM;
// 	acc.filter.data[y] = (float)(FILT_TMP[A_Y]) / (float)FILTER_NUM;
// 	acc.filter.data[z] = (float)(FILT_TMP[A_Z]) / (float)FILTER_NUM;

// 	acc.accValue.data[x] = acc.filter.data[x] * accValueFector;
// 	acc.accValue.data[y] = acc.filter.data[y] * accValueFector;
// 	acc.accValue.data[z] = acc.filter.data[z] * accValueFector;
// }

void ImuCalc::TDT_gyroFilter()
{
	u8 i;
	int32_t FILT_TMP[FILTER_NUM] = {0};

	for (i = FILTER_NUM - 1; i >= 1; i--)
	{
		FILT_BUF[G_X][i] = FILT_BUF[G_X][i - 1];
		FILT_BUF[G_Y][i] = FILT_BUF[G_Y][i - 1];
		FILT_BUF[G_Z][i] = FILT_BUF[G_Z][i - 1];
	}

	FILT_BUF[G_X][0] = gyro.calibration.data[x];
	FILT_BUF[G_Y][0] = gyro.calibration.data[y];
	FILT_BUF[G_Z][0] = gyro.calibration.data[z];

	for (i = 0; i < FILTER_NUM; i++)
	{
		FILT_TMP[G_X] += FILT_BUF[G_X][i];
		FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
		FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
	}

	gyro.filter.data[x] = (float)(FILT_TMP[G_X]) / (float)FILTER_NUM;
	gyro.filter.data[y] = (float)(FILT_TMP[G_Y]) / (float)FILTER_NUM;
	gyro.filter.data[z] = (float)(FILT_TMP[G_Z]) / (float)FILTER_NUM;

	gyro.dps.data[x] = gyro.filter.data[x] * gyroDpsFector;
	gyro.dps.data[y] = gyro.filter.data[y] * gyroDpsFector;
	gyro.dps.data[z] = gyro.filter.data[z] * gyroDpsFector;

	gyro.radps.data[x] = gyro.dps.data[x] * ANGLE_TO_RAD;
	gyro.radps.data[y] = gyro.dps.data[y] * ANGLE_TO_RAD;
	gyro.radps.data[z] = gyro.dps.data[z] * ANGLE_TO_RAD;
}

bool ImuCalc::gyroAutoCalibration()
{
	
	if (runningTimes == 0)
	{
		gyro.dynamicSum.data[x] = 0;
		gyro.dynamicSum.data[y] = 0;
		gyro.dynamicSum.data[z] = 0;
		gyro.offset_max.data[x] = -32768;
		gyro.offset_max.data[y] = -32768;
		gyro.offset_max.data[z] = -32768;
		gyro.offset_min.data[x] = 32767;
		gyro.offset_min.data[y] = 32767;
		gyro.offset_min.data[z] = 32767;
	}

	if (gyro.origin.data[x] > gyro.offset_max.data[x])
		gyro.offset_max.data[x] = gyro.origin.data[x];
	if (gyro.origin.data[y] > gyro.offset_max.data[y])
		gyro.offset_max.data[y] = gyro.origin.data[y];
	if (gyro.origin.data[z] > gyro.offset_max.data[z])
		gyro.offset_max.data[z] = gyro.origin.data[z];

	if (gyro.origin.data[x] < gyro.offset_min.data[x])
		gyro.offset_min.data[x] = gyro.origin.data[x];
	if (gyro.origin.data[y] < gyro.offset_min.data[y])
		gyro.offset_min.data[y] = gyro.origin.data[y];
	if (gyro.origin.data[z] < gyro.offset_min.data[z])
		gyro.offset_min.data[z] = gyro.origin.data[z];

	gyro.dynamicSum.data[x] += gyro.origin.data[x];
	gyro.dynamicSum.data[y] += gyro.origin.data[y];
	gyro.dynamicSum.data[z] += gyro.origin.data[z];

	runningTimes++;

	if (gyro.offset_max.data[x] - gyro.offset_min.data[x] > 30 ||
		gyro.offset_max.data[y] - gyro.offset_min.data[y] > 30 ||
		gyro.offset_max.data[z] - gyro.offset_min.data[z] > 30)
	{
		runningTimes = 0;
	}

	if (runningTimes >= GYRO_Auto_Calibration_Times)
	{
		gyro.offset.data[x] = (float)(gyro.dynamicSum.data[x]) / runningTimes;
		gyro.offset.data[y] = (float)(gyro.dynamicSum.data[y]) / runningTimes;
		gyro.offset.data[z] = (float)(gyro.dynamicSum.data[z]) / runningTimes;
		runningTimes = 0;
		return true;
	}
	return false;
}
float err_JS[3];
float intergra_JS[3];
uint64_t ImuCalc::TDT_IMU_update(bool onlyAngleIntegral)
{
	float half_T = cycle.getCycleT() / 2.0f;
	gyroAccUpdate(); // 获取原始数据
	uint64_t readImuTime = getSysTimeUs();
	caliSolve(); // 数据校准与零偏校准

#if AUTO_CALIBRATION == 1 // 自动校准陀螺仪漂移值
	gyroAutoCalibration();
#endif
	ImuCalc::gyroAccUpdate(); // 进行单位换算
	if (sixCaliFalg)		  // 六面校准，校准加速度计零偏
	{
		sixCalibration();
		memcpy(&lastAcc, &acc, sizeof(acc));
		return readImuTime;
	}
	float norm;
	float ex, ey, ez;

	if (gyroUseFilter) // 陀螺仪滑动窗口滤波
	{
		TDT_gyroFilter();
	}
	setKbuf();
	float gx = gyro.radps.data[x];
	float gy = gyro.radps.data[y];
	float gz = gyro.radps.data[z];
	float ax = acc.accValue.data[x];
	float ay = acc.accValue.data[y];
	float az = acc.accValue.data[z];
	float g1x = gyro.dps.data[x];
	float g1y = gyro.dps.data[y];
	float g1z = gyro.dps.data[z];
#if USE_AHRS_LIB

	float gyroLibIn[3] = {g1x, g1y, g1z};
	float accLibIn[3] = {ax, ay, az};
	if (onlyAngleIntegral)
	{
		accLibIn[0] = 0;
		accLibIn[1] = 0;
		accLibIn[2] = 0;
	}
	float angleLibOut[3] = {0};
	// 航资参考系统数据更新，具体声明参考ahrs_lib.h
	AHRS_update(AHRS_data.ins_quat, half_T * 2, gyroLibIn, accLibIn, AHRS_data.mag);
	// 记录上一次的值
	memcpy(&AHRS_data.LastAngle, &AHRS_data.nowAngle, sizeof(Angle));
	// 航资参考系统获取角度，具体声明参考ahrs_lib.h
	get_angle(AHRS_data.ins_quat, angleLibOut, angleLibOut + 1, angleLibOut + 2);
	// 单位换算
	AHRS_data.nowAngle.roll = angleLibOut[2] * RAD_TO_ANGLE;
	AHRS_data.nowAngle.pitch = angleLibOut[1] * RAD_TO_ANGLE;
	AHRS_data.nowAngle.yaw = angleLibOut[0] * RAD_TO_ANGLE;
	gravite = get_carrier_gravity();
	// 过圈处理
	ImuCrossRoundHandle(AHRS_data.LastAngle, AHRS_data.nowAngle, AHRS_data.Angle, AHRS_data.round);
#if !COMPARE_SELF_LIB
	memcpy(&Angle, &AHRS_data.Angle, sizeof(Angle));
#endif
#endif
	// 使用库并且比较
#if !USE_AHRS_LIB || (USE_AHRS_LIB && COMPARE_SELF_LIB)
	AngleNoZero.pitch += gy * 2 * half_T;
	AngleNoZero.roll += gx * 2 * half_T;
	AngleNoZero.yaw += gz * 2 * half_T;

	// acc数据归一化
	norm = my_sqrt(ax * ax + ay * ay + az * az);

	if (norm)
	{
		// acc数据归一化
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;

		if (!onlyAngleIntegral)
		{
			float vx, vy, vz; //(r系到b系的第三列)
			// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
			vx = 2 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2); // 四元素中xyz的表示
			vy = 2 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3);
			vz = 1 - 2 * (quaternion.q1 * quaternion.q1 + quaternion.q2 * quaternion.q2);

			// error is sum of cross product between reference direction of fields and direction measured by sensors
			ex = asin(ay * vz - az * vy); // 向量外积在相减得到差分就是误差
			ey = asin(az * vx - ax * vz);
			ez = asin(ax * vy - ay * vx);
			err_JS[0] = ex*1000;
			err_JS[1] = ey*1000;
			err_JS[2] = ez*1000;
			nowMoveState = judgeMove();//判断移动 清积分
			if(nowMoveState == 0)
			{
				imuKp = Kp *1;
				imuKi = Ki *1;
			}
			else
			{
				imuKp = Kp;
				imuKi = Ki;
			}
			err.exInt += ex * Ki * 2 * half_T; // 对误差进行积分
			err.eyInt += ey * Ki * 2 * half_T;
			err.ezInt += ez * Ki * 2 * half_T;
			
			// 积分限幅
			err.exInt = LIMIT(err.exInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
			err.eyInt = LIMIT(err.eyInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
			err.ezInt = LIMIT(err.ezInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
			
			if(lastMoveState == 1&& nowMoveState == 0)
			{
				err.ezInt = 0;
				err.eyInt = 0;
				err.exInt = 0;
			}

			lastMoveState = nowMoveState;
			intergra_JS[0] = err.exInt*1000;
			intergra_JS[1] = err.eyInt*1000;
			intergra_JS[2] = err.ezInt*1000;
			// adjusted gyroscope measurements
			gx = gx + imuKp * (ex + err.exInt);
			gy = gy + imuKp * (ey + err.eyInt);
			gz = gz + imuKp * (ez + err.ezInt);
		}

		// integrate quaternion rate and normalise						   //四元素的微分方程
		insQuat tmp_q = {0};
		tmp_q.q0 = quaternion.q0 + (-quaternion.q1 * gx - quaternion.q2 * gy - quaternion.q3 * gz) * half_T;
		tmp_q.q1 = quaternion.q1 + (quaternion.q0 * gx + quaternion.q2 * gz - quaternion.q3 * gy) * half_T;
		tmp_q.q2 = quaternion.q2 + (quaternion.q0 * gy - quaternion.q1 * gz + quaternion.q3 * gx) * half_T;
		tmp_q.q3 = quaternion.q3 + (quaternion.q0 * gz + quaternion.q1 * gy - quaternion.q2 * gx) * half_T;

		quaternion.q0 = tmp_q.q0;
		quaternion.q1 = tmp_q.q1;
		quaternion.q2 = tmp_q.q2;
		quaternion.q3 = tmp_q.q3;

		// normalise quaternion
		norm = my_sqrt(quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1 + quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
		quaternion.q0 = quaternion.q0 / norm;
		quaternion.q1 = quaternion.q1 / norm;
		quaternion.q2 = quaternion.q2 / norm;
		quaternion.q3 = quaternion.q3 / norm;

		memcpy(&LastAngle, &nowAngle, sizeof(Angle));

		nowAngle.yaw = fast_atan2(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3, -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) * 57.295780f;
		nowAngle.roll = fast_atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1, -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.295780f;
		nowAngle.pitch = asin(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.295780f;

		ImuCrossRoundHandle(LastAngle, nowAngle, Angle, round);
	}
#endif

#if USE_AHRS_LIB && COMPARE_SELF_LIB
	angleFromLib = AHRS_data.Angle;
	angleFromSelf = Angle;
#endif
	return readImuTime;
}

void ImuCalc::ImuCrossRoundHandle(eulerAngle &LastAngle, eulerAngle &nowAngle, eulerAngle &Angle, angleRound &round)
{
	// 过圈
	if (nowAngle.yaw - LastAngle.yaw > 180)
	{
		round.roundYaw--;
	}
	else if (nowAngle.yaw - LastAngle.yaw < -180)
	{
		round.roundYaw++;
	}

	Angle.yaw = round.roundYaw * 360 + nowAngle.yaw;
	Angle.pitch = nowAngle.pitch;
	Angle.roll = nowAngle.roll;
	//	Angle.readTime = gyro.readTime;
}

void ImuCalc::gyroAccUpdate()
{
	acc.accValue.data[x] = acc.calibration.data[x] * accValueFector;
	acc.accValue.data[y] = acc.calibration.data[y] * accValueFector;
	acc.accValue.data[z] = acc.calibration.data[z] * accValueFector;

	gyro.dps.data[x] = gyro.calibration.data[x] * gyroDpsFector;
	gyro.dps.data[y] = gyro.calibration.data[y] * gyroDpsFector;
	gyro.dps.data[z] = gyro.calibration.data[z] * gyroDpsFector;

	gyro.radps.data[x] = gyro.dps.data[x] * ANGLE_TO_RAD;
	gyro.radps.data[y] = gyro.dps.data[y] * ANGLE_TO_RAD;
	gyro.radps.data[z] = gyro.dps.data[z] * ANGLE_TO_RAD;
}

void ImuCalc::caliSolve()
{
	for (uint8_t i = 0; i < 3; i++)
	{
		// 角速度坐标转换
		gyro.calibration.data[i] =
			(gyro.origin.data[0] - gyro.offset.data[0]) * gyroScaleFactor.data[i][0] +
			(gyro.origin.data[1] - gyro.offset.data[1]) * gyroScaleFactor.data[i][1] +
			(gyro.origin.data[2] - gyro.offset.data[2]) * gyroScaleFactor.data[i][2];

		// 加速度坐标转换
		acc.calibration.data[i] =
			(acc.origin.data[0] - sixCaliOffset[0]) / sixCaliFector[0] * accScaleFactor.data[i][0] +
			(acc.origin.data[1] - sixCaliOffset[1]) / sixCaliFector[1] * accScaleFactor.data[i][1] +
			(acc.origin.data[2] - sixCaliOffset[2]) / sixCaliFector[2] * accScaleFactor.data[i][2];
	}
}

void ImuCalc::sixCalibration()
{
	// 加速度平方和
	sixCail.powOfacc = powf(acc.origin.data[0], 2) + powf(acc.origin.data[1], 2) + powf(acc.origin.data[2], 2);
	// 通过当前加速度和角速度判断是否在移动
	if (sixCail.powOfacc > (sixCail.g * 3) * (sixCail.g * 3) || sixCail.powOfacc < (sixCail.g * 0.1) * (sixCail.g * 0.1) || ABS(gyro.origin.data[0]) > 1000 || ABS(gyro.origin.data[1]) > 1000 || ABS(gyro.origin.data[2]) > 1000) // 运动中
	{
		sixCail.nowCauculateNum = 0;
		sixCail.waitNextAxis = true;
		sixCail.isMoving = true;
		memset(sixCail.accSixSum, 0, sizeof(sixCail.accSixSum));
		sixCail.movingJudge = 1000;
	}
	else if (sixCail.isMoving)
	{
		sixCail.movingJudge--;
	}

	if (sixCail.movingJudge < 0)
	{
		sixCail.isMoving = false;
	}
	// 如果不是在移动过程中且不是刚结束上一个轴的校准
	if (!sixCail.isMoving && sixCail.waitNextAxis)
	{
		// 判断当前的校准轴
		sixCail.nowCaliAxis = ABS(acc.origin.data[0]) > ABS(acc.origin.data[1]) ? (ABS(acc.origin.data[0]) > ABS(acc.origin.data[2]) ? 0 : 2) : (ABS(acc.origin.data[1]) > ABS(acc.origin.data[2]) ? 1 : 2);

		// 通过值的正负号区分储acc存位置
		if (acc.origin.data[sixCail.nowCaliAxis] > 0)
		{
			sixCail.nowCaliAxis = sixCail.nowCaliAxis * 2;
		}
		else
		{
			sixCail.nowCaliAxis = sixCail.nowCaliAxis * 2 + 1;
		}
		// 如果当前轴已经校准过久不再校准
		if (sixCail.accSixOffset[sixCail.nowCaliAxis] != 0)
		{
			return;
		}
		// 如果在校准过程中移动则停止校准
		if (abs(acc.origin.data[(int)(sixCail.nowCaliAxis / 2)] - lastAcc.origin.data[(int)(sixCail.nowCaliAxis / 2)]) > 100)
			return;
		// 循环累加1000次数据
		sixCail.accSixSum[sixCail.nowCaliAxis] += acc.origin.data[(int)(sixCail.nowCaliAxis / 2)];
		sixCail.gyroSixSum[sixCail.nowCaliAxis] += gyro.origin.data[(int)(sixCail.nowCaliAxis / 2)];
		sixCail.nowCauculateNum++;			   // 记录次数
		boardLed.setError(0, LedES_BlinkFast); // 记录过程中LED快闪
		if (sixCail.nowCauculateNum > 1000)
		{
			boardLed.setError(0, LedES_BlinkSlow); // 完成记录LED慢闪
			sixCail.waitNextAxis = false;
			// 计算均值
			sixCail.accSixOffset[sixCail.nowCaliAxis] = sixCail.accSixSum[sixCail.nowCaliAxis] / sixCail.nowCauculateNum + ((float)(sixCail.accSixSum[sixCail.nowCaliAxis] % sixCail.nowCauculateNum)) / sixCail.nowCauculateNum;
			sixCail.gyroSixOffset[sixCail.nowCaliAxis] = sixCail.gyroSixSum[sixCail.nowCaliAxis] / sixCail.nowCauculateNum + ((float)(sixCail.gyroSixSum[sixCail.nowCaliAxis] % sixCail.nowCauculateNum)) / sixCail.nowCauculateNum;
			sixCail.totalCnt++;		   // 记录已经校准了多少个轴
			if (sixCail.totalCnt >= 6) // 当六面校准完成进行计算和记录
			{
				sixCaliFalg = 0; // 结束校准
				for (u8 i = 0; i < 3; i++)
				{
					// 缩放量计算
					sixCaliFector[i] = (2 * sixCail.g) / (ABS(sixCail.accSixOffset[2 * i]) + ABS(sixCail.accSixOffset[2 * i + 1]));
					// 偏移量计算
					sixCaliOffset[i] = (sixCail.accSixOffset[2 * i] + sixCail.accSixOffset[2 * i + 1]) / 2;
					// 陀螺仪校准量
					gyroCaliOffset[i] = (sixCail.gyroSixOffset[2 * i] + sixCail.gyroSixOffset[2 * i + 1]) / 2;
				}
				// 看门狗复位时间1.5s
				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
				auto oldIWDG_P = IWDG->PR;
				auto oldIWDG_RL = IWDG->RLR;
				IWDG_SetPrescaler(IWDG_Prescaler_64); // 设置IWDG分频系数
				IWDG_SetReload(750);				  // 设置IWDG装载值
				IFlash.save();
				// 恢复看门狗时间
				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
				IWDG_SetPrescaler(oldIWDG_P);				  // 设置IWDG分频系数
				IWDG_SetReload(oldIWDG_RL);					  // 设置IWDG装载值
				iwdgFeed();									  // reload
				__set_FAULTMASK(1);							  // 关闭所有中断
				NVIC_SystemReset();							  // 复位
				while (1)
				{
				} // 仅等待复位
			}
		}
	}
}

bool ImuCalc::judgeMove()
{
	float nowAcc = sqrt(powf(acc.accValue.data[0],2)+powf(acc.accValue.data[0],2)+powf(acc.accValue.data[0],2));
	if(nowAcc>9.87*1.3||gyro.dps.data[0]>200||gyro.dps.data[1]>200||gyro.dps.data[2]>200)
	{
		moveJudCnt = 0;
		return 1;
	}
	else
	{
		if(moveJudCnt>500)
		{
			return 0;
		}
		else
		{
			moveJudCnt++;
			return 1;
		}
	}
}
void ImuCalc::setKbuf()
{
	  if (custom_RecvStruct.lqrKChange == 1)
    {
        memcpy(&KTest, custom_RecvStruct.lqrK, sizeof(KTest));
        // 看门狗复位时间1.5s
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
        auto oldIWDG_P = IWDG->PR;
        auto oldIWDG_RL = IWDG->RLR;
        IWDG_SetPrescaler(IWDG_Prescaler_64); // 设置IWDG分频系数
        IWDG_SetReload(750);                  // 设置IWDG装载值
        IFlash.save();
        // 恢复看门狗时间
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
        IWDG_SetPrescaler(oldIWDG_P);                 // 设置IWDG分频系数
        IWDG_SetReload(oldIWDG_RL);                   // 设置IWDG装载值
        iwdgFeed();                                   // reload
        __set_FAULTMASK(1);                           // 关闭所有中断
        NVIC_SystemReset();                           // 复位
        while (1)
        {
        } // 仅等待复位
			custom_RecvStruct.lqrKChange = 0;
    }
}