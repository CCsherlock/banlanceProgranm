#include "ut_motor.h"
#include "dbus.h"


/**
 * @brief 电机控制
 * @param T 期望关节的输出力矩（Nm）
 * @param W 期望关节速度[rad/s]
 * @param Pos 输出轴的角度 [deg]
 * @param K_P 关节刚度系数
 * @param K_W 关节速度系数
 * @note 输入数据针对输出轴，函数带数据发送，带温度保护
 */
void UT_Motor::motorCtrl(float T,float W,float Pos,float K_P,float K_W)
{
    MOTOR_send.T = T * motorInfo.dir * REDUCTION_RATIO;
    MOTOR_send.W = W * motorInfo.dir * REDUCTION_RATIO;
    MOTOR_send.Pos = (Pos * motorInfo.dir - motorPosOffset) * REDUCTION_RATIO * RAD_PER_DEG;
    MOTOR_send.K_P = K_P;
    MOTOR_send.K_W = K_W;

	if(MOTOR_recv.Temp > 100) // 电机温度超过100，切断输出，准备吃席
	{
		MOTOR_send.K_P = 0;
		MOTOR_send.K_W = 0;
		motorTempErr = true;
	}
//	else if(MOTOR_recv.Temp > 60) // 电机温度超60触发强温度保护，大幅降低参数
//	{
//		MOTOR_send.K_P *= LIMIT((100-MOTOR_recv.Temp)/80.0f,0,1.0f) * 0.5f;
//		MOTOR_send.K_W *= LIMIT((100-MOTOR_recv.Temp)/80.0f,0,1.0f) * 0.5f;
//		motorTempErr = true;
//	}
//	else if(MOTOR_recv.Temp > 40) // 电机温度超过40触发弱温度保护，线性降低参数
//	{
//		MOTOR_send.K_P *= LIMIT((100-MOTOR_recv.Temp)/60.0f,0,1.0f);
//		MOTOR_send.K_W *= LIMIT((100-MOTOR_recv.Temp)/60.0f,0,1.0f);
//		motorTempErr = true;
//	}
	else
	{
		motorTempErr = false;
	}
	
//    if(deforceFlag || (!judgement.jgmtOffline && judgement.powerHeatData.chassisVolt < 10)) // 模拟底盘断电时电机脱力
//    if(deforceFlag || (!judgement.jgmtOffline && !judgement.gameRobotStatus.mainsPowerChassisOutput)) // 模拟底盘断电时电机脱力
//        deforce();
//    else
//        onforce();

    motorTxData_Handle();
}


/**
 * @brief 发送数据处理
 * @note 具有离线保护，限制调用频率，否则容易误触离线判定
 */
void UT_Motor::motorTxData_Handle()
{
    if(motorReadyFlag == 0 && !motorLostFlag)
        msgLostTimes++;
    if(msgLostTimes > MaxToleranceLostNum)
        motorLostFlag = 1; //连续丢包，认为电机离线

    SendStruct[motorInfo.ch].head.motorID = motorInfo.motorID;
    SendStruct[motorInfo.ch].Mdata.mode = motorLostFlag ? 0 : MOTOR_send.mode; //电机离线时依旧发送，但为脱力控制

    SendStruct[motorInfo.ch].Mdata.T = MOTOR_send.T*256;
    SendStruct[motorInfo.ch].Mdata.W = MOTOR_send.W*128;
    SendStruct[motorInfo.ch].Mdata.Pos = (int)((MOTOR_send.Pos/6.2832f)*16384.0);
    SendStruct[motorInfo.ch].Mdata.K_P = MOTOR_send.K_P*2048;
    SendStruct[motorInfo.ch].Mdata.K_W = MOTOR_send.K_W*1024;
	
	RS485_Send_Data(motorInfo.ch);//串口发送
    motorReadyFlag = 0;
}


/**
 * @brief 返回校正后的输出轴角度
 * @return 校正后的角度，离线时返回0
 */
float UT_Motor::caledAgl()
{
	if(motorLostFlag)
		return 0;
	else
		return (MOTOR_recv.Pos / RAD_PER_DEG / REDUCTION_RATIO + motorPosOffset) * motorInfo.dir;
}


/**
 * @brief 电机回传数据处理
 * @note 需将每一个处理放到对应的接收中断里
 */
void UT_Motor::motorRxData_Handle(Recv_Struct_t* rs)
{
    if(rs->head.motorID != motorInfo.motorID)
        return; //ID不匹配，不进行处理
    if(rs->CRCdata != crc32_core((uint32_t*)rs, 18))
        return; //CRC不正确，不进行处理

    MOTOR_recv.motor_id = rs->head.motorID;
    MOTOR_recv.mode = rs->Mdata.mode;
    MOTOR_recv.Temp = rs->Mdata.Temp;
    MOTOR_recv.MError = rs->Mdata.MError;

    MOTOR_recv.T = ((float)rs->Mdata.T) / 256.0f;
    MOTOR_recv.W = ((float)rs->Mdata.W) / 128.0f;
    MOTOR_recv.LW = rs->Mdata.LW;
    MOTOR_recv.Acc = (int)rs->Mdata.Acc;
    MOTOR_recv.Pos = 6.2832f*((float)rs->Mdata.Pos) / 16384;

    MOTOR_recv.gyro[0] = ((float)rs->Mdata.gyro[0]) * 0.00107993176f;
    MOTOR_recv.gyro[1] = ((float)rs->Mdata.gyro[1]) * 0.00107993176f;
    MOTOR_recv.gyro[2] = ((float)rs->Mdata.gyro[2]) * 0.00107993176f;
    MOTOR_recv.acc[0] = ((float)rs->Mdata.acc[0]) * 0.0023911132f;
    MOTOR_recv.acc[1] = ((float)rs->Mdata.acc[1]) * 0.0023911132f;
    MOTOR_recv.acc[2] = ((float)rs->Mdata.acc[2]) * 0.0023911132f;

    motorReadyFlag = 1;
    motorLostFlag = 0;
    msgLostTimes = 0;
}


/**
 * @brief 电机脱力
 */
void UT_Motor::deforce()
{
    MOTOR_send.mode = 0;
}


/**
 * @brief 电机上力
 */
void UT_Motor::onforce()
{
    MOTOR_send.mode = 10;//上力如此简单
}
		

/**
 * @brief 宇树电机类构造器
 * @param id 电机的ID
 * @param ch 电机挂载的485通道
 * @param dir 电机的正方向校准
 */
UT_Motor::UT_Motor(uint8_t id, uint8_t ch, int8_t dir)
{
    motorInfo.motorID = id;
    motorInfo.ch = ch;
    motorInfo.dir = (dir == -1)?-1:1;

    MOTOR_send.mode = 0;
}

