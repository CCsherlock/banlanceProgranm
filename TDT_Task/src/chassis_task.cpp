#include "chassis_task.h"
#include "dbus.h"
#include "filter.h"
#include "ErrorTest.h"
#include "instableCheck_task.h"
#if defined SMALL_MODEL
Motor *chssisMotor[2];
Motor *legMotor[2];
Lpf2p encodeSpeedLeftFilter;
Lpf2p encodeSpeedRightFilter;
#elif defined BIG_MODEL
CyberGear *chssisMotor[2];
CyberGear *legMotor[2];
Lpf2p thetaSpeedFilter[2];
#endif
Chassis::Chassis(/* args */)
{
}
/**
 * @brief 底盘电机初始化
 *
 */
#if defined SMALL_MODEL
int legZero[2] = {1340, 1280};
#elif defined BIG_MODEL
int legZero[2] = {0, 0};
#endif

void Chassis::chassisInit()
{
#if defined SMALL_MODEL
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
    encodeSpeedLeftFilter.SetCutoffFreq(2000, 5);
    encodeSpeedRightFilter.SetCutoffFreq(2000, 5);
#elif defined BIG_MODEL
    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        chssisMotor[i] = new CyberGear(CAN1, 0x71 + i, 0x103 + i, i, Motion_mode);
        chssisMotor[i]->initMotor();
        legMotor[i] = new CyberGear(CAN1, 0x73 + i, 0x101 + i, i, Motion_mode);
        legMotor[i]->initMotor();
				thetaSpeedFilter[i].SetCutoffFreq(2000,10);
    }
#endif
}
/**
 * @brief 底盘动力电机力矩输出
 *
 * @param torque
 */
float temp;
void Chassis::chassisCtrlTorque(float torque[2])
{
#if OUTPUT_TEST
    if (deforceFlag)
    {
        motorMode = DEFORCE;
    }
    else
    {
        motorMode = RUNNING;
    }
#else
    if (deforceFlag || instableFlag)
    {
        motorMode = DEFORCE;
    }
    else
    {
        motorMode = RUNNING;
    }
#endif

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
            chssisMotor[i]->ctrlTorque(torque[i] * chassisOutputDir[i]);
#elif defined BIG_MODEL
            if (chssisMotor[i]->motorInfo.motor_mode != RUN_MODE)
            {
							for(uint8_t j = 0; j<10; j++)
							{
							 chssisMotor[i]->enableMotor();
							}
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
#if OUTPUT_TEST
    if (deforceFlag)
    {
        motorMode = DEFORCE;
    }
    else
    {
        motorMode = RUNNING;
    }
#else
    if (deforceFlag || instableFlag)
    {
        motorMode = DEFORCE;
    }
    else
    {
        motorMode = RUNNING;
    }
#endif
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
            legMotor[i]->ctrlTorque(torque[i] * legOutputDir[i]);
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
    chassisSpeed[LEFT] = chssisMotor[LEFT]->motorInfo.motor_fdb.speed * chassisFbDir[LEFT] * ROBOT_WHEEL_RADIO / 1000.0;    // 单位  m/s
    chassisSpeed[RIGHT] = chssisMotor[RIGHT]->motorInfo.motor_fdb.speed * chassisFbDir[RIGHT] * ROBOT_WHEEL_RADIO / 1000.0; // 单位  m/s
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
    chassisAngel[LEFT] = chssisMotor[LEFT]->motorInfo.motor_fdb.angle * chassisFbDir[LEFT];    // 单位 rad
    chassisAngel[RIGHT] = chssisMotor[RIGHT]->motorInfo.motor_fdb.angle * chassisFbDir[RIGHT]; // 单位 rad
#endif
    return chassisAngel;
}
/**
 * @brief 关节电机速度反馈
 *
 * @return float* 速度结构体 单位 RPM
 */
float encodeBuffer[50];
SlideWindow encodeLeftWindow(50, 50, encodeBuffer);
SlideWindow encodeRightWindow(50, 50, encodeBuffer);
float *Chassis::getLegSpeed()
{
#if defined SMALL_MODEL
    // legSpeed[LEFT] = legMotor[LEFT]->canInfo.speedFromEncoder * legFbDir[LEFT];                                 // 单位 rad/s
    // legSpeed[RIGHT] = legMotor[RIGHT]->canInfo.speedFromEncoder * legFbDir[RIGHT];                              // 单位 rad/s
    legSpeed[LEFT] = encodeSpeedLeftFilter.Apply(legMotor[LEFT]->canInfo.speedFromEncoder) * legFbDir[LEFT];    // 单位 rad/s
    legSpeed[RIGHT] = encodeSpeedLeftFilter.Apply(legMotor[RIGHT]->canInfo.speedFromEncoder) * legFbDir[RIGHT]; // 单位 rad/s
#elif defined BIG_MODEL
//    legSpeed[LEFT] = legMotor[LEFT]->megSpeed * chassisFbDir[LEFT];    // 电机反馈速度 rad/s
//    legSpeed[RIGHT] = legMotor[RIGHT]->megSpeed * chassisFbDir[RIGHT]; // 电机反馈速度 rad/s
    legSpeed[LEFT] = thetaSpeedFilter[LEFT].Apply(legMotor[LEFT]->megSpeed * chassisFbDir[LEFT]);    // 电机反馈速度 rad/s
    legSpeed[RIGHT] = thetaSpeedFilter[RIGHT].Apply(legMotor[RIGHT]->megSpeed * chassisFbDir[RIGHT]); // 电机反馈速度 rad/s
//   legSpeed[LEFT] = encodeLeftWindow.slideWindowFilter(legMotor[LEFT]->megSpeed);                  // 编码器反馈速度 rad/s
//   legSpeed[RIGHT] = encodeRightWindow.slideWindowFilter(legMotor[RIGHT]->megSpeed);                // 编码器反馈速度 rad/s
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
    legAngel[LEFT] = legMotor[LEFT]->megAngle * chassisFbDir[LEFT];    // 单位 rad
    legAngel[RIGHT] = legMotor[RIGHT]->megAngle * chassisFbDir[RIGHT]; // 单位 rad
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
float encode_last;
uint64_t timeFrom, timeLast;
float legSpeedCal(float encode)
{
    float radpsSpeed;
    timeFrom = getSysTimeUs();
    radpsSpeed = (encode - encode_last) / (float)(timeFrom - timeLast);
    encode_last = encode;
    timeLast = timeFrom;
    return radpsSpeed;
}