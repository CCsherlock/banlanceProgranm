#include "crossStand.h"

float standSpeedP = 5;
float standTurnP = 3;
uint8_t CrossStandMode::intoModeRun(RobotMotion _modeLast)
{
			if(!modeInitFlag)
		{
			modeInit();
			modeInitFlag = true;
		}
    switch (_modeLast)
    {
    case CROSS_STAND:
        robotCtrl.chassisSpeed = 0; // m/s
				robotCtrl.chassisYaw = bmi088Cal->Angle.yaw;
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
        robotCtrl.chassisSpeed = 0; // m/s
				robotCtrl.chassisYaw = bmi088Cal->Angle.yaw;
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
        robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT])/2, thetaStart[LEFT], thetaEnd[LEFT]);
        robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT])/2, thetaStart[RIGHT], thetaEnd[RIGHT]);
        if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish&& ABS(balance.angleFb[LEFT] - robotCtrl.bodyTheta[LEFT])<0.1 &&ABS(balance.angleFb[RIGHT] - robotCtrl.bodyTheta[RIGHT])<0.1)
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
			if(!modeInitFlag)
		{
			modeInit();
			modeInitFlag = true;
		}
		robotCtrl.chassisSpeed = 0;  // m/s
    robotCtrl.chassisYaw = bmi088Cal->Angle.yaw;
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
void CrossStandMode::reset()
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
void CrossStandMode::modeInit()
{
}