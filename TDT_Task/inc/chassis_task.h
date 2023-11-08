#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_
#include "board.h"
#include "ut_motor.h"
#include "motor.h"
class Chassis
{
private:
    /* data */
public:
    Chassis(/* args */);
    void chassisInit();
    Motor *chssisMotor[2];
    UT_Motor *legMotor[2];
    void chassisCtrlTorque(float torque[2]);
    void legCtrlTorque(float torque[2]);
};





#endif