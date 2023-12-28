#include "mode_task.h"
#include "lqrCtrl_task.h"
#include "dbus.h"
DeforceMode deforceMode;
SitMode sitMode;
CrossStandMode standMode;
#if defined SMALL_MODEL
float speedP = 10;
float turnP = 5;
#else
float speedP = 1;
float turnP = 1;
#endif
int RunMode::modeNum = ALL_MODE_NUM;
RunMode **RunMode::modeList = 0;
RunMode::RunMode(RobotMotion _thisMode)
{
    if (modeList == 0)
    {
        modeList = (RunMode **)malloc(sizeof(RunMode *));
    }
    else
    {
        modeList = (RunMode **)realloc(modeList, sizeof(RunMode *) * ALL_MODE_NUM);
    }
    modeList[_thisMode] = this;
    thisMode = _thisMode;
}
void DeforceMode::inModeRun()
{
    robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
    robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
    robotCtrl.chassisTurnSpeed = 0;
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    transeResetFlag = false;
}
uint8_t DeforceMode::intoModeRun(RobotMotion _modeLast)
{
    switch (_modeLast)
    {
    default:
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
        robotCtrl.bodyPitch = 0;
        break;
    }
    transeOverFlag = true;
    return transeOverFlag;
}

uint8_t SitMode::intoModeRun(RobotMotion _modeLast)
{
    switch (_modeLast)
    {
    case SIT:
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyPitch = 0;
        if (!recodeTranseFlag)
        {
            thetaEnd[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;
            thetaEnd[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG;
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
    case DEFORCE:
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
        robotCtrl.bodyPitch = pitchRamp.ramp((0 - balance.fiFb), balance.fiFb, 0);
        if (pitchRamp.curveFinish)
        {
            pitchRamp.reset();
            transeOverFlag = true;
        }
        break;
    case CROSS_STAND:
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
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
    default:
        transeOverFlag = false;
        break;
    }
    return transeOverFlag;
}
void SitMode::inModeRun()
{
    robotCtrl.chassisTurnSpeed = (RC.Key.CH[0] / 660.f) * ROBOT_MAX_V * turnP;                                  // m/s
    robotCtrl.chassisSpeed[LEFT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * speedP + robotCtrl.chassisTurnSpeed;  // m/s
    robotCtrl.chassisSpeed[RIGHT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * speedP - robotCtrl.chassisTurnSpeed; // m/s
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;                          // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG;                        // rad
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
void SitMode::reset()
{
    thetaRamp[LEFT].reset();
    thetaRamp[RIGHT].reset();
    pitchRamp.reset();
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = true;
}
uint8_t CrossStandMode::intoModeRun(RobotMotion _modeLast)
{
    switch (_modeLast)
    {
    case CROSS_STAND:
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyPitch = 0;
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
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyPitch = 0;
        if (!recodeTranseFlag)
        {
            thetaEnd[LEFT] = jumpThetaCal(balance.angleFb[LEFT], 180, 180, 1) * RAD_PER_DEG;
            thetaEnd[RIGHT] = jumpThetaCal(balance.angleFb[RIGHT], 180, 180, -1) * RAD_PER_DEG;
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
    default:
        transeOverFlag = false;
        break;
    }
    return transeOverFlag;
}
void CrossStandMode::inModeRun()
{
    robotCtrl.chassisSpeed[LEFT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * speedP;          // m/s
    robotCtrl.chassisSpeed[RIGHT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * speedP;         // m/s
    robotCtrl.chassisTurnSpeed = (RC.Key.CH[0] / 660.f) * ROBOT_MAX_V * turnP;             // m/s
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
void CrossStandMode::reset()
{
    thetaRamp[LEFT].reset();
    thetaRamp[RIGHT].reset();
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = true;
}