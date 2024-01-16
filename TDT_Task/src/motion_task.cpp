#include "motion_task.h"
#include "imu_task.h"
#include "lqrCtrl_task.h"
#include "instableCheck_task.h"
using namespace RCS;
Motion robotMotion;
RampCurve ThetaUpRamp;
#define N0_STATE_TRANSE 0
Motion::Motion()
{
}
/**
 * @brief 机器人根据遥控器进行状态切换
 * @mode DEFORCE SIT SLEEP CROSS_STAND
 */
void Motion::motionModeSwitch()
{
    if (deforceFlag || instableFlag)
    {
        robotMode = DEFORCE;
        robotMode_last = robotMode;
				runModeJudge();
        return;
    }
    if (RC.Key.SW2 == Mid && RC.Key.SW1 == Mid) // 上力置中
    {
        robotMode = SIT;
    }
    else if (RC.Key.SW2 == Mid && RC.Key.SW1 == Up)
    {
        robotMode = CROSS_STAND;
    }
    runModeJudge();
}
void Motion::runModeJudge()
{
#if N0_STATE_TRANSE
	RunMode::modeList[robotMode]->inModeRun();
#else
    if (robotMode != robotMode_last)
    {
        modeChangeFlag = true;
    }
    for (uint8_t mode = 0; mode < ALL_MODE_NUM; mode++)
    {
        /* code */
        if(mode != robotMode)
        {
            RunMode::modeList[mode]->reset();
        }
    }
    if (modeChangeFlag)
    {
        /* code */
        RunMode::modeList[robotMode]->intoModeRun(robotMode_last);
        if (RunMode::modeList[robotMode]->transeOverFlag)
        {
            modeChangeFlag = 0;
            robotMode_last = robotMode;
        }
    }
    else
    {
        /* code */
        RunMode::modeList[robotMode]->inModeRun();
        robotMode_last = robotMode;
    }	
#endif
}
/**
 * @brief 速度设定
 *
 */
void Motion::chassisSpeedCtrl()
{
    balance.speedSet = RunMode::modeList[robotMode]->robotCtrl.chassisSpeed;
}
/**
 * @brief 内圈角度设定
 *
 */
void Motion::bodyThetaCtrl()
{
    balance.angleSet[LEFT] = RunMode::modeList[robotMode]->robotCtrl.bodyTheta[LEFT];
    balance.angleSet[RIGHT] = RunMode::modeList[robotMode]->robotCtrl.bodyTheta[RIGHT];
}
/**
 * @brief 身体俯仰角设定
 *
 */
void Motion::bodyPitchCtrl()
{
    balance.fiSet = (RunMode::modeList[robotMode]->robotCtrl.bodyPitch + fiOffset) * RAD_PER_DEG; // rad
}
void Motion::yawCtrl()
{
    balance.yawSet = RunMode::modeList[robotMode]->robotCtrl.chassisYaw; //rad
}
void motionLoop()
{
    robotMotion.motionModeSwitch(); // 设置机器人整体状态
    robotMotion.chassisSpeedCtrl(); // 设置底盘速度
    robotMotion.bodyThetaCtrl();    // 设置机器人内圈角度
    robotMotion.bodyPitchCtrl();    // 设置机体俯角
		robotMotion.yawCtrl();
}