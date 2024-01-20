#include "crossStand.h"
float standTurnP = 3;
/**
 * @brief 进入交错站姿模式切换状态控制
 * 
 * @param _modeLast 进入前的状态
 * @return uint8_t 
 */
uint8_t CrossStandMode::intoModeRun(RobotMotion _modeLast)
{
    if (!modeInitFlag)
    {
        modeInit();
        modeInitFlag = true;
    }
    switch (_modeLast)
    {
    case CROSS_STAND:
        robotCtrl.chassisSpeed = 0; // m/s
        robotCtrl.chassisYaw = balance.yawFb;
        robotCtrl.bodyPitch = 0;
        balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM);
        if (!recodeTranseFlag)
        {
            thetaEnd[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;
            thetaEnd[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG;
            thetaStart[LEFT] = balance.angleFb[LEFT];
            thetaStart[RIGHT] = balance.angleFb[RIGHT];
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            recodeTranseFlag = 1;
        }
        robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT]), thetaStart[LEFT], thetaEnd[LEFT]);
        robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT]), thetaStart[RIGHT], thetaEnd[RIGHT]);
        if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish)
        {
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            transeOverFlag = true;
        }
        break;
    case SIT:
        robotCtrl.chassisSpeed = 0; // m/s
        robotCtrl.chassisYaw = balance.yawFb;
        robotCtrl.bodyPitch = 0;
        if (!recodeTranseFlag)
        {
            thetaEnd[LEFT] = jumpThetaCal(balance.angleFb[LEFT], 180, 180, -1) * RAD_PER_DEG;
            thetaEnd[RIGHT] = jumpThetaCal(balance.angleFb[RIGHT], 180, 180, 1) * RAD_PER_DEG;
            thetaStart[LEFT] = balance.angleFb[LEFT];
            thetaStart[RIGHT] = balance.angleFb[RIGHT];
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            recodeTranseFlag = 1;
        }
        robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT]) / 1.6, thetaStart[LEFT], thetaEnd[LEFT]);
        robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT]) / 1.5, thetaStart[RIGHT], thetaEnd[RIGHT]);
        if (ABS(balance.angleFb[LEFT] - thetaEnd[LEFT]) < 0.8 && ABS(balance.angleFb[RIGHT] - thetaEnd[RIGHT]) < 0.8)
        {
            balance.roboLqr->setNowParam(balance.roboLqr->UP_PARAM);
        }
        else
        {
            balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM);
        }
        if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish && ABS(balance.angleFb[LEFT] - thetaEnd[LEFT]) < 0.2 && ABS(balance.angleFb[RIGHT] - thetaEnd[RIGHT]) < 0.2)
        {
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            balance.roboLqr->setNowParam(balance.roboLqr->UP_PARAM);
            transeOverFlag = true;
        }
        break;
    default:
        transeOverFlag = false;
        break;
    }
    return transeOverFlag;
}
void CrossStandMode::inModeRun()
{
    if (!modeInitFlag)
    {
        modeInit();
        modeInitFlag = true;
    }
    robotCtrl.chassisSpeed = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V / 5; // m/s
    balance.roboLqr->setNowParam(balance.roboLqr->UP_PARAM);
    robotCtrl.chassisYaw += (RC.Key.CH[0] / 660.f) * ROBOT_MAX_W / 5;
    //		robotCtrl.chassisYaw = balance.yawFb;
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
void CrossStandMode::reset()
{
    if (!modeInitFlag)
    {
        modeInit();
        modeInitFlag = true;
    }
    thetaRamp[LEFT].reset();
    thetaRamp[RIGHT].reset();
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = true;
}
void CrossStandMode::modeInit()
{
}