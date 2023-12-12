#include "chassis_task.h"
#include "dbus.h"
#if defined SMALL_MODEL
Motor *chssisMotor[2];
Motor *legMotor[2];
#elif defined BIG_MODEL
CyberGear *chssisMotor[2];
CyberGear *legMotor[2];
#endif
Chassis::Chassis(/* args */)
{
}
/**
 * @brief 底盘电机初始化
 *
 */
#if defined SMALL_MODEL
int legZero[2] = {1429, 1362};
#elif defined BIG_MODEL
int legZero[2] = {0, 0};
#endif

void Chassis::chassisInit()
{
    chssisMotor[LEFT] = new Motor(M3508, CAN1, 0x201);
    chssisMotor[LEFT]->setMotorTorqueCoff(12970);
    legMotor[LEFT] = new Motor(GM6020, CAN1, 0X206);
    legMotor[LEFT]->setZeroValue(legZero[LEFT]);
    legMotor[LEFT]->setMotorTorqueCoff(25000);
    chssisMotor[RIGHT] = new Motor(M3508, CAN1, 0x202);
    chssisMotor[RIGHT]->setMotorTorqueCoff(12970);
    legMotor[RIGHT] = new Motor(GM6020, CAN1, 0X205);
    legMotor[RIGHT]->setZeroValue(legZero[RIGHT]);
    legMotor[RIGHT]->setMotorTorqueCoff(25000);
    //    for (u8 i = 0; i < 2; i++)
    //    {
    ///* code */
    // #if defined SMALL_MODEL
    //         chssisMotor[i] = new Motor(M3508, CAN1, 0x201 + i);
    //         chssisMotor[i]->setMotorTorqueCoff(12970);
    //         legMotor[i] = new Motor(GM6020, CAN1, 0X205 + i);
    //         legMotor[i]->setZeroValue(legZero[i]);
    //         legMotor[i]->setMotorTorqueCoff(25000);
    // #elif defined BIG_MODEL
    //         chssisMotor[i] = new CyberGear(CAN1, 0x7F + i, i + 1, Motion_mode);
    //         chssisMotor[i]->initMotor();
    //         legMotor[i] = new CyberGear(CAN1, 0x81 + i, i + 1, Motion_mode);
    //         legMotor[i]->initMotor();
    // #endif
    //     }
}
/**
 * @brief 底盘动力电机力矩输出
 *
 * @param torque
 */
float temp;
void Chassis::chassisCtrlTorque(float torque[2])
{
	if(deforceFlag)
	{
		motorMode = DEFORCE;
	}
	else
	{
		motorMode = RUNNING;
	}
    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        switch (motorMode)
        {
        case DEFORCE:
#if defined SMALL_MODEL
            chssisMotor[i]->ctrlTorque(0);
#elif defined BIG_MODEL
            chssisMotor[i]->stopMotor(0);
#endif
            break;
        case RUNNING:
#if defined SMALL_MODEL
            chssisMotor[i]->ctrlTorque(torque[i]*chassisOutputDir[i]);
#elif defined BIG_MODEL
            if (chssisMotor[i]->motorInfo.motor_mode != RUN_MODE)
            {
                chssisMotor[i]->enableMotor();
            }
            chssisMotor[i]->motorCtrlMode(torque[i], 0, 0, 0, 0);
#endif
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
	if(deforceFlag)
	{
		motorMode = DEFORCE;
	}
	else
	{
		motorMode = RUNNING;
	}
    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        switch (motorMode)
        {
        case DEFORCE:
#if defined SMALL_MODEL
            legMotor[i]->ctrlTorque(0);
#elif defined BIG_MODEL
            legMotor[i]->stopMotor(0);
#endif
            break;
        case RUNNING:
#if defined SMALL_MODEL
            legMotor[i]->ctrlTorque(torque[i]* legOutputDir[i]);
#elif defined BIG_MODEL
            if (legMotor[i]->motorInfo.motor_mode != RUN_MODE)
            {
                legMotor[i]->enableMotor();
            }
            legMotor[i]->motorCtrlMode(torque[i], 0, 0, 0, 0);
#endif
            break;
        default:
            break;
        }
    }
}
/**
 * @brief 底盘电机速度电机反馈
 *
 * @return float* 速度结构体
 */
float *Chassis::getChassisSpeed()
{
#if defined SMALL_MODEL
    chassisSpeed[LEFT] = chssisMotor[LEFT]->canInfo.speed * chassisFbDir[LEFT];    // 单位  RPM
    chassisSpeed[RIGHT] = chssisMotor[RIGHT]->canInfo.speed * chassisFbDir[RIGHT]; // 单位  RPM
#elif defined BIG_MODEL
    chassisSpeed[LEFT] = chssisMotor[LEFT]->motorInfo.motor_fdb.speed * chassisFbDir[LEFT];
    chassisSpeed[RIGHT] = chssisMotor[RIGHT]->motorInfo.motor_fdb.speed * chassisFbDir[RIGHT];
#endif
    return chassisSpeed;
}
/**
 * @brief 底盘电机位置反馈
 *
 * @return float* 位置结构体
 */
float *Chassis::getChassisAngel()
{
#if defined SMALL_MODEL
    chassisAngel[LEFT] = chssisMotor[LEFT]->canInfo.totalAngle_f * chassisFbDir[LEFT];    // 单位 °
    chassisAngel[RIGHT] = chssisMotor[RIGHT]->canInfo.totalAngle_f * chassisFbDir[RIGHT]; // 单位 °
#elif defined BIG_MODEL
    chassisAngel[LEFT] = chssisMotor[LEFT]->motorInfo.motor_fdb.angle * chassisFbDir[LEFT];
    chassisAngel[RIGHT] = chssisMotor[RIGHT]->motorInfo.motor_fdb.angle * chassisFbDir[RIGHT];
#endif
    return chassisAngel;
}
/**
 * @brief 关节电机速度反馈
 *
 * @return float* 速度结构体 单位 RPM
 */
float *Chassis::getLegSpeed()
{
#if defined SMALL_MODEL
    legSpeed[LEFT] = legMotor[LEFT]->canInfo.speed * legFbDir[LEFT];    // 单位 RPM
    legSpeed[RIGHT] = legMotor[RIGHT]->canInfo.speed * legFbDir[RIGHT]; // 单位 RPM
#elif defined BIG_MODEL
    chassisAngel[LEFT] = legMotor[LEFT]->motorInfo.motor_fdb.speed * chassisFbDir[LEFT];
    chassisAngel[RIGHT] = legMotor[RIGHT]->motorInfo.motor_fdb.speed * chassisFbDir[RIGHT];
#endif
    return legSpeed;
}
/**
 * @brief 关节电机位置反馈
 *
 * @return float* 位置结构体 单位 °
 */
float *Chassis::getLegAngel()
{
#if defined SMALL_MODEL
    legAngel[LEFT] = legMotor[LEFT]->canInfo.totalAngle_f * legFbDir[LEFT];    // 单位 °
    legAngel[RIGHT] = legMotor[RIGHT]->canInfo.totalAngle_f * legFbDir[RIGHT]; // 单位 °
#elif defined BIG_MODEL
    chassisAngel[LEFT] = legMotor[LEFT]->motorInfo.motor_fdb.angle * chassisFbDir[LEFT];
    chassisAngel[RIGHT] = legMotor[RIGHT]->motorInfo.motor_fdb.angle * chassisFbDir[RIGHT];
#endif
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