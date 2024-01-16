#include "sitMode.h"
float sitSpeedP = -10;
float sitTurnP = 3;
float sitTurnPidP = 0.01;
uint8_t SitMode::intoModeRun(RobotMotion _modeLast)
{
    if (!modeInitFlag)
    {
        modeInit();
        modeInitFlag = true;
    }
    switch (_modeLast)
    {
    case SIT:
        robotCtrl.chassisSpeed = 0; // m/s
				robotCtrl.chassisYaw = bmi088Cal->Angle.yaw * RAD_PER_DEG;
        robotCtrl.bodyPitch = 0;
        if (!recodeTranseFlag)
        {
            thetaEnd[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;
            thetaEnd[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG;
            thetaStart[LEFT] = balance.angleFb[LEFT];
            thetaStart[RIGHT] = balance.angleFb[RIGHT];
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            fiStart = balance.fiFb / RAD_PER_DEG;
            fiEnd = fiStart;
            pitchRamp.reset();
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
    case DEFORCE:
        robotCtrl.chassisSpeed = 0; // m/s
				robotCtrl.chassisYaw = bmi088Cal->Angle.yaw * RAD_PER_DEG;
        if (!recodeTranseFlag)
        {
            thetaEnd[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;
            thetaEnd[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG;
            thetaStart[LEFT] = balance.angleFb[LEFT];
            thetaStart[RIGHT] = balance.angleFb[RIGHT];
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            fiStart = balance.fiFb / RAD_PER_DEG;
            fiEnd = 0;
            pitchRamp.reset();
            recodeTranseFlag = 1;
        }
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
        robotCtrl.bodyPitch = pitchRamp.ramp((fiEnd - fiStart)*2, fiStart, fiEnd);
        if (pitchRamp.curveFinish)
        {
            pitchRamp.reset();
            transeOverFlag = true;
        }
        break;
    case CROSS_STAND:
        robotCtrl.chassisSpeed = 0; // m/s
				robotCtrl.chassisYaw = bmi088Cal->Angle.yaw * RAD_PER_DEG;
        robotCtrl.bodyPitch = 0;
        if (!recodeTranseFlag)
        {
            thetaEnd[LEFT] = jumpThetaCal(balance.angleFb[LEFT], 0, 180, -1) * RAD_PER_DEG;
            thetaEnd[RIGHT] = jumpThetaCal(balance.angleFb[RIGHT], 0, 180, 1) * RAD_PER_DEG;
            thetaStart[LEFT] = balance.angleFb[LEFT];
            thetaStart[RIGHT] = balance.angleFb[RIGHT];
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            recodeTranseFlag = 1;
        }
        robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT]) * 2, thetaStart[LEFT], thetaEnd[LEFT]);
        robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT]) * 2, thetaStart[RIGHT], thetaEnd[RIGHT]);
        if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish)
        {
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            transeOverFlag = true;
        }
        break;
    case JUMP:
        robotCtrl.chassisSpeed = 0; // m/s
				robotCtrl.chassisYaw = bmi088Cal->Angle.yaw * RAD_PER_DEG;
        robotCtrl.bodyPitch = 0;
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
        transeOverFlag = true;
        break;
    default:
        transeOverFlag = false;
        break;
    }
    return transeOverFlag;
}
void SitMode::inModeRun()
{
    if (!modeInitFlag)
    {
        modeInit();
        modeInitFlag = true;
    }
    robotCtrl.chassisYaw += (RC.Key.CH[0] / 660.f) * sitTurnP * 0.005;    // m/s
    robotCtrl.chassisSpeed = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * sitSpeedP; // m/s
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;                                        // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG;                                      // rad
		bodyThetaCalculate();
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
void SitMode::reset()
{
    if (!modeInitFlag)
    {
        modeInit();
        modeInitFlag = true;
    }
    thetaRamp[LEFT].reset();
    thetaRamp[RIGHT].reset();
    pitchRamp.reset();
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = true;
}
void SitMode::modeInit()
{
}
void SitMode::speedPidCalculate()
{

}
void SitMode::bodyThetaCalculate()
{
//    thetaBySpeed[LEFT] = (RC.Key.CH[3] / 660.f) * 90 ;
//    thetaBySpeed[RIGHT] = (RC.Key.CH[3] / 660.f) * 90 ;
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0 + thetaBySpeed[LEFT]) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0 + thetaBySpeed[RIGHT]) * RAD_PER_DEG; // rad
}