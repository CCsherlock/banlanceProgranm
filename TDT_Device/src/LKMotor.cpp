#include "LKMotor.h"
#include "can.h"
#include "my_math.h"
CanTxMsg LKNomalCanSendMsg;
CanTxMsg LKCommandCanSendMsg;
uint8_t canSentMode = 0; // 0：nomalCtrl 1: commandCtrl

LKMotor::LKMotor(CAN_TypeDef *_Canx, uint8_t _motorNumber)
{
    thisCanType = _Canx;
    motorNumber = _motorNumber;
    motorStdId = ZEOR_MOTOR_IDENDTIFY + motorNumber;
}

void LKMotor::startMotor()
{
    canSentMode = 1;
    motorMode = LK_START;
    LKCommandCanSendMsg.StdId = motorStdId;
    LKCommandCanSendMsg.Data[0] = MOTOR_COMMAND_START;
    canTx(LKCommandCanSendMsg.Data, CAN1, 0, LKCommandCanSendMsg.StdId);
}
void LKMotor::stopMotor()
{
    canSentMode = 1;
    motorMode = LK_DEFORCE;
    LKCommandCanSendMsg.StdId = motorStdId;
    LKCommandCanSendMsg.Data[0] = MOTOR_COMMAND_CLOSE;
    canTx(LKCommandCanSendMsg.Data, CAN1, 0, LKCommandCanSendMsg.StdId);
}
void LKMotor::motorCtrlTorque(float torque)
{
    torque = LIMIT(torque, -MAX_CTRL_TORUQE, MAX_CTRL_TORUQE);
    int16_t currunt;
    currunt = LIMIT((torque * TORQUE_TO_CURRUNT_MF7015), -MAX_CTRL_IQ_NUMBER, MAX_CTRL_IQ_NUMBER);
    motorCtrlCurrunt(currunt);
}
void LKMotor::motorCtrlSpeed(int32_t speed) // 0.01 dps/LSB
{
    uint8_t buffer[4];
    buffer[0] = *int32ToU8(speed);
    buffer[1] = *(int32ToU8(speed) + 1);
    buffer[2] = *(int32ToU8(speed) + 2);
    buffer[3] = *(int32ToU8(speed) + 3);
    LKNomalCanSendMsg.Data[0] = MOTOR_SINGLE_COMMAND_SPEED;
    LKNomalCanSendMsg.Data[4] = buffer[0];
    LKNomalCanSendMsg.Data[5] = buffer[1];
    LKNomalCanSendMsg.Data[6] = buffer[2];
    LKNomalCanSendMsg.Data[7] = buffer[3];
    LKNomalCanSendMsg.StdId = motorStdId;
    canTx(LKNomalCanSendMsg.Data, CAN1, 0, LKNomalCanSendMsg.StdId);
}
void LKMotor::motorCtrlSpeed(float speed)
{
    int32_t speedInt;
    speedInt = (int)speed * 100;
    motorCtrlSpeed(speedInt);
}
void LKMotor::motorCtrlPossion(int32_t possion)
{
    uint8_t buffer[4];
    buffer[0] = *int32ToU8(possion);
    buffer[1] = *(int32ToU8(possion) + 1);
    buffer[2] = *(int32ToU8(possion) + 2);
    buffer[3] = *(int32ToU8(possion) + 3);
    LKNomalCanSendMsg.Data[0] = MOTOR_SINGLE_COMMAND_POSSION;
    LKNomalCanSendMsg.Data[4] = buffer[0];
    LKNomalCanSendMsg.Data[5] = buffer[1];
    LKNomalCanSendMsg.Data[6] = buffer[2];
    LKNomalCanSendMsg.Data[7] = buffer[3];
    LKNomalCanSendMsg.StdId = motorStdId;
    canTx(LKNomalCanSendMsg.Data, CAN1, 0, LKNomalCanSendMsg.StdId);
}
void LKMotor::motorCtrlPossion(float possion)
{
    int32_t possionInt;
    possionInt = (int)possion * 100;
    motorCtrlPossion(possionInt);
}
void LKMotor::motorCtrlCurrunt(int16_t currunt)
{
    canSentMode = 0;
    uint8_t buffer[2];
    buffer[0] = *int16ToU8(currunt);
    buffer[1] = *(int16ToU8(currunt) + 1);
    LKNomalCanSendMsg.Data[0] = MOTOR_SINGLE_COMMAND_TORQUE;
    LKNomalCanSendMsg.Data[4] = buffer[0];
    LKNomalCanSendMsg.Data[5] = buffer[1];
    LKNomalCanSendMsg.StdId = motorStdId;
    canTx(LKNomalCanSendMsg.Data, CAN1, 0, LKNomalCanSendMsg.StdId);
}
void LKMotor::motorDataHandler(CanRxMsg *canRxData)
{
    motorInfo.temperature = canRxData->Data[1];                                            // 1℃/LSB
    motorInfo.torqueI = uint8Toint16(canRxData->Data[2], canRxData->Data[3]);              //-2048-2048  --> -33A-33A
    motorInfo.torqueA = ((float)motorInfo.torqueI / 2048) * 33 / TORQUE_TO_CURRUNT_MF7015; // Nm
    motorInfo.speed = uint8Toint16(canRxData->Data[4], canRxData->Data[5]);                // 1dps/LSB
    motorInfo.speed_rad = motorInfo.speed * RAD_PER_DEG;                                   // radps
    motorInfo.encoder = uint8Toint16(canRxData->Data[6], canRxData->Data[7]);              // 0-16383
    motorInfo.possition = ((float)motorInfo.encoder / 16384) * 2 * PI;                     // rad
    motorInfo.lostCnt = 0;
}
void LKCanSendMessage()
{
    if (canSentMode == 0)
    {
        canTx(LKNomalCanSendMsg.Data, CAN1, 0, LKNomalCanSendMsg.StdId);
    }
    else if (canSentMode == 1)
    {
        canTx(LKCommandCanSendMsg.Data, CAN1, 0, LKCommandCanSendMsg.StdId);
    }
}

void LkMotorLostCheck(LKMotor *motor)
{
    motor->motorInfo.lostCnt++;
    if (motor->motorInfo.lostCnt > 100)
    {
        motor->motorInfo.lostFlag = 1;
    }
}

FormatTrans2 FT2;
FormatTrans3 FT3;
uint8_t *int16ToU8(int16_t int16)
{
    FT2.I16t = int16;
    return FT2.U8t;
}

int16_t uint8Toint16(uint8_t u81, uint8_t u82)
{
    FT2.U8t[0] = u81;
    FT2.U8t[1] = u82;
    return FT2.I16t;
}

uint8_t *int32ToU8(int32_t int32)
{
    FT3.I32t = int32;
    return FT3.U8t;
}