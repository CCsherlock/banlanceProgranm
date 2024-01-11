#include "CyberGear.h"
#include "can.h"
#include "my_math.h"
/*******************************************************************************
 * @function     : 浮点数转4字节函数
 * @param        : 浮点数
 * @return       : 4字节数组
 * @description  : IEEE 754 协议
 *******************************************************************************/
uint8_t byte[4];
uint8_t *Float_to_Byte(float f)
{
    unsigned long longdata = 0;
    longdata = *(unsigned long *)&f;
    byte[0] = (longdata & 0xFF000000) >> 24;
    byte[1] = (longdata & 0x00FF0000) >> 16;
    byte[2] = (longdata & 0x0000FF00) >> 8;
    byte[3] = (longdata & 0x000000FF);
    return byte;
}
float u8toflaot(uint8_t u4, uint8_t u3, uint8_t u2, uint8_t u1)
{
    FormatTrans FT;
    FT.U[3] = u4;
    FT.U[2] = u3;
    FT.U[1] = u2;
    FT.U[0] = u1;
    return FT.F;
}
/*******************************************************************************
 * @function     : 小米电机回文16位数据转浮点
 * @param        : 1. 16位回文 2.对应参数下限 3.对应参数上限 4. 参数位数
 * @return       : 参数对应浮点数
 * @description  : None
 *******************************************************************************/
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/**
 * @brief  float转int，数据打包用
 * @param  x float数值
 * @param  x_min float数值的最小值
 * @param  x_max float数值的最大值
 * @param  bits  int的数据位数
 * @retval null
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief Construct a new Cyber Gear:: Cyber Gear object
 *
 * @param _Canx   CAN1 / CAN2
 * @param _Ext_ID
 */
CyberGear::CyberGear(CAN_TypeDef *_Canx, uint8_t _Ext_ID, uint8_t _Meg_ID, int Motor_Num, float mode)
{
    myCan_x = _Canx;
    _extID = _Ext_ID;
    _megBoardID = _Meg_ID;
    _motorNum = Motor_Num;
    _runMode = mode;
    motorInfo.lostFlag = 1;
    motorInfo.lostCnt = 1000;
}
CyberGear::CyberGear(CAN_TypeDef *_Canx, uint8_t _Ext_ID, int Motor_Num, float mode)
{
    myCan_x = _Canx;
    _extID = _Ext_ID;
    _megBoardID = 0x200;
    _motorNum = Motor_Num;
    _runMode = mode;
    motorInfo.lostFlag = 1;
    motorInfo.lostCnt = 1000;
}
/*******************************************************************************
 * @function     : 电机参数初始化
 * @param        : 1. 电机结构体 2.电机CANID 3.电机编号 4.电机工作模式（1.运动模式 2. 位置模式 3. 速度模式 4. 电流模式）
 * @return       : None
 * @description  : 负责初始化电机 CANID 电机编号 电机的工作模式
 *******************************************************************************/
void CyberGear::initMotor()
{
    motorInfo.phcan = myCan_x;
    motorInfo.EXT_ID.motor_id = _extID;
    enableMotor();
}
/*******************************************************************************
 * @function     : 提取电机回复帧扩展ID中的电机CANID
 * @param        : 电机回复帧中的扩展CANID
 * @return       : 电机ID
 * @description  : 取扩展ID的8-15位
 *******************************************************************************/
uint32_t CyberGear::getMotorID(uint32_t CAN_ID_Frame)
{
    motorInfo.motor_mode = GET_ID_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_GetID;
    motorInfo.EXT_ID.motor_id = 0;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = 0;
    memset(motorInfo.txdata, 0, sizeof(motorInfo.txdata));
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
    return (CAN_ID_Frame & 0xFFFF) >> 8;
}
void CyberGear::getMotorState(uint16_t CAN_ID_Data)
{
    memcpy(&motorInfo.motor_Id_mode, &CAN_ID_Data, sizeof(motorInfo.motor_Id_mode));
}
/*******************************************************************************
 * @function     : 使能电机
 * @param        : 对应控制电机结构体
 * @return       : None
 * @description  : 使能电机
 *******************************************************************************/
void CyberGear::enableMotor()
{
    motorInfo.motor_mode = RUN_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_MotorEnable;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = Master_CAN_ID;
    memset(motorInfo.txdata, 0, sizeof(motorInfo.txdata));
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}
/*******************************************************************************
 * @function     : 停止电机
 * @param        : 1.对应控制电机结构体 2.清除错误位（0 不清除 1清除）
 * @return       : None
 * @description  : 使能电机
 *******************************************************************************/
void CyberGear::stopMotor(uint8_t clear_error)
{
    motorInfo.motor_mode = DEFORCE_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_MotorStop;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = Master_CAN_ID;
    memset(motorInfo.txdata, 0, sizeof(motorInfo.txdata));
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}
void CyberGear::motorCtrlMode(float torque, float MechPosition, float speed, float kp, float kd)
{
    motorInfo.motor_mode = RUN_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_MotionControl;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = float_to_uint(torque, T_MIN, T_MAX, 16);

    motorInfo.txdata[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    motorInfo.txdata[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16);
    motorInfo.txdata[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
    motorInfo.txdata[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
    motorInfo.txdata[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    motorInfo.txdata[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    motorInfo.txdata[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    motorInfo.txdata[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}
/*******************************************************************************
 * @function     : 获取电机参数
 * @param        : 
 * @return       : None
 * @description  : None
 *******************************************************************************/
void CyberGear::getSingleParam(uint16_t paramList)
{
}
/*******************************************************************************
 * @function     : 写入电机参数
 * @param        : 1.写入参数对应地址 2.写入参数值
 * @return       : None
 * @description  : None
 *******************************************************************************/
void CyberGear::setMotorParameter(uint16_t index, uint8_t data[4])
{
    motorInfo.motor_mode = GET_PARAM_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_SetSingleParameter;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = Master_CAN_ID;
    memcpy(&motorInfo.txdata[0], &index, 2);
    memcpy(&motorInfo.txdata[4], data, 4);
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}
/*******************************************************************************
 * @function     : 写入电机参数
 * @param        : 1.写入参数对应地址 2.写入参数值
 * @return       : None
 * @description  : None
 *******************************************************************************/
void CyberGear::setMotorParameter(uint16_t index, float data)
{
    motorInfo.motor_mode = GET_PARAM_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_SetSingleParameter;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = Master_CAN_ID;
    memcpy(&motorInfo.txdata[0], &index, 2);
    uint8_t temp[4];
    memcpy(temp, Float_to_Byte(data), sizeof(temp));
    memcpy(&motorInfo.txdata[4], temp, 4);
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}
/*******************************************************************************
 * @function     : 写入电机参数
 * @param        : 1.写入参数对应地址 2.写入参数值
 * @return       : None
 * @description  : None
 *******************************************************************************/
void CyberGear::setMotorParameter(uint16_t index, uint8_t data)
{
    motorInfo.motor_mode = GET_PARAM_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_SetSingleParameter;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = Master_CAN_ID;
    memcpy(&motorInfo.txdata[0], &index, 2);
    motorInfo.txdata[4] = data;
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}
/*******************************************************************************
 * @function     : 设置电机机械零点
 * @param        : %2
 * @return       : %None
 * @description  : %4
 *******************************************************************************/
void CyberGear::setZeroPos()
{
    motorInfo.EXT_ID.mode = Communication_Type_SetPosZero;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = Master_CAN_ID;
    memset(motorInfo.txdata, 0, sizeof(motorInfo.txdata));
    motorInfo.txdata[0] = 1;
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}
/*******************************************************************************
 * @function     : 设置电机CANID
 * @param        : %2
 * @return       : %3
 * @description  : %4
 *******************************************************************************/
void CyberGear::setCANID(uint8_t Target_ID)
{
    motorInfo.motor_mode = GET_ID_MODE;
    motorInfo.EXT_ID.mode = Communication_Type_CanID;
    motorInfo.EXT_ID.motor_id = _extID;
    motorInfo.EXT_ID.res = 0;
    motorInfo.EXT_ID.data = Target_ID << 8 | Master_CAN_ID;
    memset(motorInfo.txdata, 0, sizeof(motorInfo.txdata));
    motorInfo.txdata[0] = 0;
    uint32_t sendId;
    memcpy(&sendId, &motorInfo.EXT_ID, sizeof(motorInfo.EXT_ID));
    canTx(motorInfo.txdata, motorInfo.phcan, sendId);
    motorInfo.lostCnt++;
}

uint8_t megCANmessege[8];
/**
 * @brief 设置编码器零点
 *
 */
void CyberGear::setMegZeroOffset()
{
    megTrans.setZeroFlag = 1;
    megCANmessege[0 + _motorNum * sizeof(megTrans)] = megTrans.setZeroFlag;
    if ((_motorNum + 1) * sizeof(megTrans) <= 8)
    {
        canTx(megCANmessege, MEG_BOARD_CANX, MEG_BOARD_CANID);
        megTrans.lostCnt++;
    }
    megTrans.setZeroFlag = 0;
}
/**
 * @brief 重启编码器
 *
 */
void CyberGear::resetMegBoard()
{
    megTrans.resetFlag = 1;
    megCANmessege[1 + _motorNum * sizeof(megTrans)] = megTrans.resetFlag;
    if ((_motorNum + 1) * sizeof(megTrans) <= 8)
    {
        canTx(megCANmessege, MEG_BOARD_CANX, MEG_BOARD_CANID);
        megTrans.lostCnt++;
    }
    megTrans.resetFlag = 0;
}
/*******************************************************************************
 * @function     : 电机回复帧数据处理函数
 * @param        : 1.对应控制电机结构体 2.数据帧 3.扩展ID帧
 * @return       : None
 * @description  : 获取角度 速度 扭矩 温度 错误代码
 *******************************************************************************/

uint16_t decode_temp_mi = 0;
void CyberGear::motorDataHandler(CanRxMsg *canRxData)
{
    memcpy(&motorInfo.EXT_ID_RX, canRxData, sizeof(motorInfo.EXT_ID_RX));
    getMotorState(motorInfo.EXT_ID_RX.data);
    motorInfo.motor_fdb.angle_temp = uint16_to_float(canRxData->Data[0] << 8 | canRxData->Data[1], MIN_P, MAX_P, 16);
    motorInfo.motor_fdb.angle = motorInfo.motor_fdb.angle_temp / RAD_PER_DEG; // ±720 °
    motorInfo.motor_fdb.speed_temp = uint16_to_float(canRxData->Data[2] << 8 | canRxData->Data[3], V_MIN, V_MAX, 16);
    motorInfo.motor_fdb.speed = motorInfo.motor_fdb.speed_temp; // ±30 rad/s
    motorInfo.motor_fdb.torque_temp = uint16_to_float(canRxData->Data[4] << 8 | canRxData->Data[5], T_MIN, T_MAX, 16);
    motorInfo.motor_fdb.torque = motorInfo.motor_fdb.torque_temp; // ±12 N/m
    motorInfo.motor_fdb.temprature_temp = (canRxData->Data[6] << 8 | canRxData->Data[7]);
    motorInfo.motor_fdb.temprature = motorInfo.motor_fdb.temprature_temp * Temp_Gain; // ℃
    motorInfo.lostCnt = 0;
    motorInfo.lostFlag = 0;
}
/**
 * @brief 小米电机编码器信息获取
 *
 * @param canRxData can接收的标准帧信息
 */
void CyberGear::megSpeedMessegeGet(CanRxMsg *canRxData)
{
    uint8_t receiveBuffer[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        receiveBuffer[i] = canRxData->Data[i];
    }
    megAngle = u8toflaot(receiveBuffer[3], receiveBuffer[2], receiveBuffer[1], receiveBuffer[0]); // rad
    for (uint8_t i = 0; i < 4; i++)
    {
        receiveBuffer[i] = canRxData->Data[i + 4];
    }
    megSpeed = u8toflaot(receiveBuffer[3], receiveBuffer[2], receiveBuffer[1], receiveBuffer[0]); // rad/s
    megSpeed_encode = (megAngle - megAngle_last) / timeIntervalFrom_f(megTimeRecode);
    megTimeRecode = getSysTimeUs() / 1e6f;
    megAngle_last = megAngle;
    megTrans.lostCnt = 0;
    megTrans.lostFlag = 0;
}
/**
 * @brief 更改当前电机ID，更改后重新初始化电机
 *
 * @param changeId
 */
void CyberGear::changeThisId(uint8_t changeId)
{
    setCANID(changeId);
    _extID = changeId;
    initMotor();
}
/**
 * @brief 小米电机丢失检测
 *
 * @param motor
 */
void cyberGearLostCheck(CyberGear *motor)
{
    motor->motorInfo.lostCnt++;
    if (motor->motorInfo.lostCnt > 100)
    {
        motor->motorInfo.lostFlag = 1;
    }
    motor->megTrans.lostCnt++;
    if (motor->megTrans.lostCnt > 100)
    {
        motor->megTrans.lostFlag = 1;
    }
}