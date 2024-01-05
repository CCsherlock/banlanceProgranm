#include "jumpMode.h"

uint8_t JumpMode::intoModeRun(RobotMotion _modeLast)
{
			if(!modeInitFlag)
		{
			modeInit();
			modeInitFlag = true;
		}
    switch (_modeLast)
    {
    case SIT:
        thetaStart[LEFT] = balance.angleFb[LEFT];
        thetaStart[RIGHT] = balance.angleFb[RIGHT];
        thetaEnd[LEFT] = jumpThetaCal(balance.angleFb[LEFT], 360, 360, 1) * RAD_PER_DEG;
        thetaEnd[RIGHT] = jumpThetaCal(balance.angleFb[RIGHT], 360, 360, 1) * RAD_PER_DEG;
        jumpFinishFlag = 0;
        transeOverFlag = 1;
        break;
    default: 
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
        robotCtrl.bodyPitch = balance.fiFb / RAD_PER_DEG;
        transeOverFlag = 0;
        break;
    }
    return transeOverFlag;
}
void JumpMode::inModeRun()
{
			if(!modeInitFlag)
		{
			modeInit();
			modeInitFlag = true;
		}
    if (!jumpFinishFlag)
    {
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyPitch = 0;
        robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT]) / 0.3, thetaStart[LEFT], thetaEnd[LEFT]);
        robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT]) / 0.3, thetaStart[RIGHT], thetaEnd[RIGHT]);
        if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish && ABS(balance.angleFb[LEFT] - robotCtrl.bodyTheta[LEFT])<0.1 &&ABS(balance.angleFb[RIGHT] - robotCtrl.bodyTheta[RIGHT])<0.1)
        {
            jumpFinishFlag = 1;
        }
    }
    else
    {
        robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
        robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
        robotCtrl.chassisTurnSpeed = 0;
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
        robotCtrl.bodyPitch = 0;
    }
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
void JumpMode::reset()
{
			if(!modeInitFlag)
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
void JumpMode::modeInit()
{
		speedPid[LEFT]	= new Pid(1);
		speedPid[RIGHT]	= new Pid(1);
		speedParam.kp = 0;
		speedParam.resultMax = 5;
		speedPid[LEFT]->paramPtr = &speedParam;
		speedPid[LEFT]->fbValuePtr[0] = &balance.speedFb[LEFT];
		speedPid[RIGHT]->paramPtr = &speedParam;
		speedPid[RIGHT]->fbValuePtr[0] = &balance.speedFb[RIGHT];
}