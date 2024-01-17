#include "deforceMode.h"

void DeforceMode::inModeRun()
{
    robotCtrl.chassisSpeed = 0; // m/s
		robotCtrl.chassisYaw = balance.yawFb;
#if defined START_FROM_UP
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
    robotCtrl.bodyPitch = balance.fiFb;
    transeOverFlag = false;
    transeResetFlag = false;
}
uint8_t DeforceMode::intoModeRun(RobotMotion _modeLast) 
{
    switch (_modeLast)
    {
    default:
        robotCtrl.chassisSpeed = 0; // m/s
				robotCtrl.chassisYaw = balance.yawFb;
#if defined START_FROM_UP
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
        robotCtrl.bodyPitch = balance.fiFb;
        break;
    }
    transeOverFlag = true;
    return transeOverFlag;
}