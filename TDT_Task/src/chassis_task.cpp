#include "chassis_task.h"
Motor *legMotor[2];
CyberGear *motorMi[2];
Chassis::Chassis(/* args */)
{
}
/**
 * @brief 底盘电机初始化
 *
 */
void Chassis::chassisInit()
{

    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        // chssisMotor[i] = new Motor(M3508, CAN1, 0x201 + i);

        // legMotor[i] = new UT_Motor(i,i,1);
        // legMotor[i] = new Motor(GM6020, CAN1, 0X205 + i);
        motorMi[i] = new CyberGear(CAN1, 0x7F + i, i + 1, Motion_mode);
        motorMi[i]->initMotor();
    }
}
/**
 * @brief 底盘动力电机力矩输出
 *
 * @param torque
 */
uint32_t torqueChange(float _t)
{
    return (uint32_t)((_t + 12) / 24.0f * 65536);
}
float temp;
void Chassis::chassisCtrlTorque(float torque[2])

    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        // chssisMotor[i]->ctrlCurrent(torque[i] * chaMotorTq2Cu * chassisOutputDir[i]);
        switch (chassisMode)
        {
        case DEFORCE:
            motorMi[i]->stopMotor(0);
            break;
        case ENABLE:
            motorMi[i]->enableMotor();
            break;
        case RUNNING:
            motorMi[i]->motorCtrlMode(temp, 0, 0, 0, 0);
            break;
        default:
            break;
        }
    }
}
/**
 * @brief 关节电机力矩输出
 *
 * @param torque
 */
void Chassis::legCtrlTorque(float torque[2])
{
    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        // legMotor[i]->motorCtrl(torque[i], 0, 0, 0, 0);
        legMotor[i]->ctrlCurrent(torque[i] * legMotorTq2Cu * legOutputDir[i]);
    }
}
/**
 * @brief 底盘电机速度电机反馈
 *
 * @return float* 速度结构体
 */
float *Chassis::getChassisSpeed()
{
    // chassisSpeed[LEFT] = chssisMotor[LEFT]->canInfo.speed * chassisFbDir[LEFT];
    // chassisSpeed[RIGHT] = chssisMotor[RIGHT]->canInfo.speed * chassisFbDir[RIGHT];
    chassisSpeed[LEFT] = motorMi[LEFT]->motorInfo.motor_fdb.speed * chassisFbDir[LEFT];
    chassisSpeed[RIGHT] = motorMi[RIGHT]->motorInfo.motor_fdb.speed * chassisFbDir[RIGHT];
    return chassisSpeed;
}
/**
 * @brief 底盘电机位置反馈
 *
 * @return float* 位置结构体
 */
float *Chassis::getChassisAngel()
{
    // chassisAngel[LEFT] = chssisMotor[LEFT]->canInfo.totalAngle_f * chassisFbDir[LEFT];
    // chassisAngel[RIGHT] = chssisMotor[RIGHT]->canInfo.totalAngle_f * chassisFbDir[RIGHT];
    chassisAngel[LEFT] = motorMi[LEFT]->motorInfo.motor_fdb.angle * chassisFbDir[LEFT];
    chassisAngel[RIGHT] = motorMi[RIGHT]->motorInfo.motor_fdb.angle * chassisFbDir[RIGHT];
    return chassisAngel;
}
/**
 * @brief 关节电机速度反馈
 *
 * @return float* 速度结构体
 */
float *Chassis::getLegSpeed()
{
    legSpeed[LEFT] = legMotor[LEFT]->canInfo.speed * legFbDir[LEFT];
    legSpeed[RIGHT] = legMotor[RIGHT]->canInfo.speed * legFbDir[RIGHT];
    return legSpeed;
}
/**
 * @brief 关节电机位置反馈
 *
 * @return float* 位置结构体
 */
float *Chassis::getLegAngel()
{
    legAngel[LEFT] = legMotor[LEFT]->canInfo.totalAngle_f * legFbDir[LEFT];
    legAngel[RIGHT] = legMotor[RIGHT]->canInfo.totalAngle_f * legFbDir[RIGHT];
    return legAngel;
}
/**
 * @brief 设置底盘动力电机输出方向
 *
 * @param left
 * @param right
 */
void Chassis::setChassisOutPutDir(int8_t left, int8_t right)
{
    chassisOutputDir[LEFT] = left;
    chassisOutputDir[RIGHT] = right;
}
/**
 * @brief 设置关机电机输出方向
 *
 * @param left
 * @param right
 */
void Chassis::setlegOutPutDir(int8_t left, int8_t right)
{
    legOutputDir[LEFT] = left;
    legOutputDir[RIGHT] = right;
}
/**
 * @brief 设置底盘动力电机反馈值方向
 *
 * @param left
 * @param right
 */
void Chassis::setChassisFbDir(int8_t left, int8_t right)
{
    chassisFbDir[LEFT] = left;
    chassisFbDir[RIGHT] = right;
}
/**
 * @brief 设置关节电机反馈方向
 *
 * @param left
 * @param right
 */
void Chassis::setLegFbDir(int8_t left, int8_t right)
{
    legFbDir[LEFT] = left;
    legFbDir[RIGHT] = right;
}