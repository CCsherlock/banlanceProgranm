#include "motion_task.h"
#include "dbus.h"
#include "imu_task.h"
#include "lqrCtrl_task.h"
using namespace RCS;
Motion::Motion()
{
}
void Motion::motionModeSwitch()
{
    if (deforceFlag)
    {
        robotMode = DEFORCE;
        robotMode_last = robotMode;
        return;
    }
    if (RC.Key.SW2 == Mid && RC.Key.SW1 == Mid) // 上力置中
    {
        robotMode = SIT;
    }
    else if (RC.Key.SW2 == Mid && RC.Key.SW1 == Down)
    {
        robotMode = SLEEP;
    }
    else if (RC.Key.SW2 == Mid && RC.Key.SW1 == Up)
    {
        robotMode = STAND;
    }
    robotMode_last = robotMode;
}
/**
 * @brief
 *
 */
void Motion::chassisSpeedCtrl()
{
    switch (robotMode)
    {
    case DEFORCE:
        robotCtrl.chassisSpeed = 0;
        break;
    case SIT:
        robotCtrl.chassisSpeed = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * 0.5;
        break;
    default:
        break;
    }
    robotCtrl.chassisSpeed = LIMIT(robotCtrl.chassisSpeed, -ROBOT_MAX_V, ROBOT_MAX_V);
    balance.speedSet[LEFT] = robotCtrl.chassisSpeed;
    balance.speedSet[RIGHT] = robotCtrl.chassisSpeed;
}
/**
 * @brief
 *
 */
void Motion::bodyThetaCtrl()
{
    switch (robotMode)
    {
    case DEFORCE:
        robotCtrl.bodyTheta = 0;
        break;
    case SIT:
        robotCtrl.bodyTheta = 0;
        break;
    case STAND:
        robotCtrl.bodyTheta = 90;
        break;
    default:
        break;
    }
    balance.angleSet[LEFT] = robotCtrl.bodyTheta;
    balance.angleSet[RIGHT] = robotCtrl.bodyTheta;
}
/**
 * @brief
 *
 */
void Motion::bodyPitchCtrl()
{
    switch (robotMode)
    {
    case DEFORCE:
        robotCtrl.bodyPitch = 0;
        break;
    case SIT:
        robotCtrl.bodyPitch = 0;
    case STAND:
        robotCtrl.bodyPitch = 0;
        break;
    default:
        robotCtrl.bodyPitch = 0;
        break;
    }
    balance.fiSet = robotCtrl.bodyPitch;
}
