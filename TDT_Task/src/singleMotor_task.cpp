#include "singleMotor_task.h"
#include "cyberGear.h"
#include "dbus.h"
CyberGear testMotor(CAN1, 0x7F, 1, Motion_mode);

struct MotorCtrlMode
{
    uint8_t setId;
    uint8_t setParam;
    uint8_t ctrlTorque;
    uint8_t ctrlSpeed;
    uint8_t ctrlPossition;
    uint8_t setTargetId;
    uint16_t paramIndex;
};

MotorCtrlMode motorCtrlmode =
    {
        .setId = 0,
        .setParam = 0,
        .ctrlTorque = 0,
        .ctrlSpeed = 0,
        .ctrlPossition = 0,
        .setTargetId = 0x74,//待更改电机ID
};
void setMotorId()
{
    testMotor.stopMotor(0);
    testMotor.changeThisId(motorCtrlmode.setTargetId);
}
void setMotorParam()
{
    testMotor.stopMotor(0);
    testMotor.setMotorParameter(Run_mode, (uint8_t)Current_mode);
}
float ctrlTestT = 0;
void setMotorTorque()
{
    if (testMotor.motorInfo.motor_mode != RUN_MODE)
    {
        testMotor.enableMotor();
    }
    testMotor.motorCtrlMode(ctrlTestT, 0, 0, 0, 0);
}
void setMotorSpeed()
{
}
void setMotorPossition()
{
}
void motorTestLoop()
{
    static uint8_t motorInit;
    if (!motorInit)
    {
        testMotor.initMotor();
        motorInit = true;
    }
    if (motorCtrlmode.setId)
    {
        setMotorId();
    }
    else if (motorCtrlmode.setParam)
    {
    }
    else if (motorCtrlmode.ctrlTorque)
    {
        setMotorTorque();
    }
}