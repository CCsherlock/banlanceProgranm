#include "sitMode.h"
#include "imu_task.h"
float sitSpeedP = 10;
float sitTurnP = 0.5;
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
        robotCtrl.chassisYaw = bmi088Cal->Angle.yaw;
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
            fiStart = balance.fiFb / RAD_PER_DEG;
            fiEnd = fiStart;
            pitchRamp.reset();
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
        robotCtrl.chassisYaw = bmi088Cal->Angle.yaw;
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
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
        robotCtrl.bodyPitch = pitchRamp.ramp((fiEnd - fiStart), fiStart, fiEnd);
        if (pitchRamp.curveFinish)
        {
            pitchRamp.reset();
            transeOverFlag = true;
        }
        break;
    case CROSS_STAND:
        robotCtrl.chassisYaw = bmi088Cal->Angle.yaw;
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
    case JUMP:
        robotCtrl.chassisYaw = bmi088Cal->Angle.yaw;
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
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
    robotCtrl.chassisTurnSpeed = -(RC.Key.CH[0] / 660.f) * sitTurnP;    // m/s
//    speedPidCalculate();
    robotCtrl.chassisSpeed[LEFT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * sitSpeedP - robotCtrl.chassisTurnSpeed;  // m/s
    robotCtrl.chassisSpeed[RIGHT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * sitSpeedP + robotCtrl.chassisTurnSpeed; // m/s
//    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;                                        // rad
//    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG;                                      // rad
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
    speedPid[LEFT] = new Pid(1);
    speedPid[RIGHT] = new Pid(1);
    yawFollowOuterPid = new Pid(3);
    speedParam[LEFT].kp = 1;
    speedParam[LEFT].ki = 0.1;
    speedParam[LEFT].integralErrorMax = 20;
    speedParam[LEFT].resultMax = 10;
    speedPid[LEFT]->paramPtr = &speedParam[LEFT];
    speedPid[LEFT]->fbValuePtr[0] = &balance.speedFb[LEFT];
    speedParam[RIGHT].kp = 1;
    speedParam[RIGHT].ki = 0.1;
    speedParam[RIGHT].integralErrorMax = 20;
    speedParam[RIGHT].resultMax = 10;
    speedPid[RIGHT]->paramPtr = &speedParam[RIGHT];
    speedPid[RIGHT]->fbValuePtr[0] = &balance.speedFb[RIGHT];

    yawFollowOuter[0].kp = 1;
    yawFollowOuter[0].ki = 0.01;
    yawFollowOuter[0].integralErrorMax = 0.5;
    yawFollowOuter[0].resultMax = 1;

    yawFollowOuter[1].kp = 0.5;
    yawFollowOuter[1].ki = 0.5;
    yawFollowOuter[1].integralErrorMax = 1;
    yawFollowOuter[1].resultMax = 5;

    yawFollowOuter[2].kp = 0.2;
    yawFollowOuter[2].ki = 1;
    yawFollowOuter[2].integralErrorMax = 2;
    yawFollowOuter[2].resultMax = 10;
    yawFollowOuterPid->paramPtr = &yawFollowOuter[0];
		for(uint8_t i = 0;i<3;i++)
		{
			yawFollowOuterPid->fbValuePtr[i] = &bmi088Cal->Angle.yaw;
		}
}
void SitMode::speedPidCalculate()
{
    robotCtrl.chassisYaw += -(RC.Key.CH[0] / 660.f) * sitTurnPidP;
    if (ABS(yawFollowOuterPid->error) > 20)
    {
        robotCtrl.chassisTurnSpeed = yawFollowOuterPid->Calculate(robotCtrl.chassisYaw, 2);
    }
    else if (20 > ABS(yawFollowOuterPid->error) && ABS(yawFollowOuterPid->error)  > 10)
    {
        robotCtrl.chassisTurnSpeed = yawFollowOuterPid->Calculate(robotCtrl.chassisYaw, 1);
    }
    else if(ABS(yawFollowOuterPid->error) < 10 )
    {
        robotCtrl.chassisTurnSpeed = yawFollowOuterPid->Calculate(robotCtrl.chassisYaw, 0);
    }
}
void SitMode::bodyThetaCalculate()
{
    thetaBySpeed[LEFT] = (RC.Key.CH[3] / 660.f) * 90 ;
    thetaBySpeed[RIGHT] = (RC.Key.CH[3] / 660.f) * 90 ;
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0 + thetaBySpeed[LEFT]) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0 + thetaBySpeed[RIGHT]) * RAD_PER_DEG; // rad
}