#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_
#include "board.h"
#include "ut_motor.h"
#include "motor.h"
#include "CyberGear.h"
class Chassis
{
private:
    /* data */
    int8_t chassisOutputDir[2] = {1, 1};
    int8_t legOutputDir[2] = {1, 1};
    int8_t chassisFbDir[2] = {1, 1};
    int8_t legFbDir[2] = {1, 1};
    enum ChassisMode
    {
        DEFORCE = 0,
        ENABLE,
        RUNNING
    };
public:
    Chassis(/* args */);
    void chassisInit();
    Motor *chssisMotor[2];

    float chassisSpeed[2];
    float chassisAngel[2];
    float legSpeed[2];
    float legAngel[2];
    void chassisCtrlTorque(float torque[2]);
    void legCtrlTorque(float torque[2]);
    float* getChassisSpeed();
    float* getChassisAngel();
    float* getLegSpeed();
    float* getLegAngel();
    void setChassisOutPutDir(int8_t left, int8_t right);
    void setlegOutPutDir(int8_t left, int8_t right);
    void setChassisFbDir(int8_t left, int8_t right);
    void setLegFbDir(int8_t left, int8_t right);
    float chaMotorTq2Cu = 1;
    float legMotorTq2Cu = 1;
    ChassisMode chassisMode = DEFORCE;
};
extern Motor *legMotor[2];
extern CyberGear *motorMi[2];
#endif