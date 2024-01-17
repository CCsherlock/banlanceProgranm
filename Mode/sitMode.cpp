#include "sitMode.h"
float sitSpeedP = -10;
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
				balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM);
        robotCtrl.chassisSpeed = 0; // m/s
        robotCtrl.chassisYaw = balance.yawFb;
        robotCtrl.bodyPitch = 0;
				robotCtrl.chassisTurnSpeed = 0;
        if (!recodeTranseFlag)
        {
#if defined START_FROM_UP
						robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
						robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
						robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
						robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
            thetaStart[LEFT] = balance.angleFb[LEFT];
            thetaStart[RIGHT] = balance.angleFb[RIGHT];
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            fiStart = balance.fiFb;
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
				balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM);
        robotCtrl.chassisSpeed = 0; // m/s
        robotCtrl.chassisYaw = balance.yawFb;
				robotCtrl.chassisTurnSpeed = 0;
        if (!recodeTranseFlag)
        {
#if defined START_FROM_UP
						robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
						robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
						robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
						robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
            thetaStart[LEFT] = balance.angleFb[LEFT];
            thetaStart[RIGHT] = balance.angleFb[RIGHT];
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            fiStart = balance.fiFb;
            fiEnd = 0;
            pitchRamp.reset();
            recodeTranseFlag = 1;
        }
#if defined START_FROM_UP
				robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
				robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
				robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
				robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
        robotCtrl.bodyPitch = pitchRamp.ramp((fiEnd - fiStart) * 2, fiStart, fiEnd);
        robotCtrl.chaTorqueKp = 0;
        if (pitchRamp.curveFinish && (ABS(balance.fiFb - fiEnd) < (30 * RAD_PER_DEG)))
        {
            pitchRamp.reset();		
            transeOverFlag = true;
            robotCtrl.chaTorqueKp = 1;
        }
        break;
    case CROSS_STAND:
				balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM);
        robotCtrl.chassisSpeed = 0; // m/s
        robotCtrl.chassisYaw = balance.yawFb;
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
    case JUMP:
				balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM);
        robotCtrl.chassisSpeed = 0; // m/s
        robotCtrl.chassisYaw = balance.yawFb;
				robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyPitch = 0;
#if defined START_FROM_UP
				robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
				robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
				robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
				robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
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
		balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM);
#if defined START_FROM_UP
		robotCtrl.chassisYaw += (RC.Key.CH[0] / 660.f)  * ROBOT_MAX_W;                   // m/s
//		robotCtrl.chassisYaw = balance.yawFb;
//		robotCtrl.chassisTurnSpeed = (RC.Key.CH[0] / 660.f)  * ROBOT_MAX_W * 10;
    robotCtrl.chassisSpeed = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V / 10;           // m/s
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
		robotCtrl.chassisYaw += (RC.Key.CH[0] / 660.f)  * ROBOT_MAX_W;                   // m/s
    robotCtrl.chassisSpeed = -(RC.Key.CH[3] / 660.f) * ROBOT_MAX_V /2;           // m/s
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
//    bodyThetaCalculate();
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
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0 + thetaBySpeed[LEFT]) * RAD_PER_DEG;    // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0 + thetaBySpeed[RIGHT]) * RAD_PER_DEG; // rad
}