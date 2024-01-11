#include "sitMode.h"
#include "imu_task.h"
float sitSpeedP = 10;
float sitTurnP = 0.1;
float sitTurnPidP = 0.1;
uint8_t SitMode::intoModeRun(RobotMotion _modeLast)
{
		if(!modeInitFlag)
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
		if(!modeInitFlag)
		{
			modeInit();
			modeInitFlag = true;
		}
//		robotCtrl.chassisTurnSpeed = -(RC.Key.CH[0] / 660.f) * sitTurnP;    // m/s
		robotCtrl.chassisYaw += -(RC.Key.CH[0] / 660.f) * sitTurnPidP;
	  robotCtrl.chassisTurnSpeed = yawFollowOuterPid->Calculate(robotCtrl.chassisYaw);
    robotCtrl.chassisSpeed[LEFT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * sitSpeedP - robotCtrl.chassisTurnSpeed * sitTurnP;  // m/s
    robotCtrl.chassisSpeed[RIGHT] = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V * sitSpeedP + robotCtrl.chassisTurnSpeed * sitTurnP; // m/s
//    robotCtrl.chassisSpeed[LEFT] = speedPid[LEFT]->Calculate(robotCtrl.chassisSpeed[LEFT]);
//    robotCtrl.chassisSpeed[RIGHT] = speedPid[RIGHT]->Calculate(robotCtrl.chassisSpeed[RIGHT]);
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
void SitMode::reset()
{
		if(!modeInitFlag)
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
		speedPid[LEFT]	= new Pid(1);
		speedPid[RIGHT]	= new Pid(1);
		yawFollowOuterPid = new Pid(1);
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
	
		yawFollowOuter.kp = 1;
		yawFollowOuter.ki = 0;
	  yawFollowOuter.integralErrorMax = 0.5;
		yawFollowOuter.resultMax = 10;
		yawFollowOuterPid->paramPtr = &yawFollowOuter;
		yawFollowOuterPid->fbValuePtr[0] = &bmi088Cal->Angle.yaw;

}