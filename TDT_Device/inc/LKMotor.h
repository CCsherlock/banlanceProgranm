#ifndef _LM_MOTOR_H_
#define _LM_MOTOR_H_
#include "board.h"

// 多电机控制指令ID
#define CTRL_COMMAND_TORQUE 0x280
#define CTRL_COMMAND_SPEED 0x281
#define CTRL_COMMAND_POSSITION 0x282
#define CTRL_COMMAND_MIX 0x288

// 多电机控制    CTRL_COMMAND_MIX
#define READ_MOTOR_STATE1 0x9A
#define CLEAR_MOTOR_ERROR 0x9B
#define READ_MOTOR_STATE2 0x9C
#define MOTOR_COMMAND_CLOSE 0x80
#define MOTOR_COMMAND_START 0x88
#define MOTOR_COMMAND_PAUSE 0x81

#define ZEOR_MOTOR_IDENDTIFY 0x140

// 单电机控制 SINGLE_MOTOR
#define MOTOR_SINGLE_COMMAND_TORQUE 0xA1
#define MOTOR_SINGLE_COMMAND_SPEED 0xA2
#define MOTOR_SINGLE_COMMAND_POSSION 0xA3

#define TORQUE_TO_CURRUNT_MF7015 10
#define MAX_CTRL_CURRUNT 32
#define MAX_CTRL_IQ_NUMBER 2000
#define MAX_CTRL_TORUQE 2.5
#define MAX_CTRL_SPEED

struct motor_fb_info
{
    /* data */
    int8_t temperature; // 温度
    int16_t torqueI;    // 转矩电流
    float torqueA;      // 扭矩电流值
    int16_t speed;      // 速度 dps
    float speed_rad;    // 速度rad/s
    uint16_t encoder;   // 编码器位置 14位 0-16383
    float possition;    // rad
    int lostCnt;
    uint8_t lostFlag;
};

enum Motor_run_mode
{
    LK_DEFORCE = 0,
    LK_START
};

class LKMotor
{
private:
    /* data */

public:
    CAN_TypeDef *thisCanType;
    uint8_t motorNumber;
    uint32_t motorStdId;
    LKMotor(CAN_TypeDef *_Canx, uint8_t _motorNumber);
    motor_fb_info motorInfo;
    Motor_run_mode motorMode;
    void startMotor();
    void stopMotor();
    void motorDataHandler(CanRxMsg *canRxData);
    void motorCtrlTorque(float torque);
    void motorCtrlCurrunt(int16_t currunt);
    void motorCtrlSpeed(int32_t speed);
    void motorCtrlSpeed(float speed);
    void motorCtrlPossion(int32_t possion);
    void motorCtrlPossion(float possion);
    uint8_t sendBuffer[2];
};

void LKCanSendMessage();
void LKCanMsgProssess();

typedef union
{
    uint8_t U8t[2];
    uint16_t U16t;
    int16_t I16t;
} FormatTrans2;

typedef union
{
    uint8_t U8t[4];
    uint16_t U16t[2];
    int16_t I16t[2];
    int32_t I32t;
} FormatTrans3;

uint8_t *int16ToU8(int16_t int16);
int16_t uint8Toint16(uint8_t u81, uint8_t u82);
uint8_t *int32ToU8(int32_t int32);
void LkMotorLostCheck(LKMotor *motor);
#endif