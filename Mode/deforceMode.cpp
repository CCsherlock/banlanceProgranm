#include "deforceMode.h"

void DeforceMode::inModeRun()
{
    robotCtrl.chassisSpeed[LEFT] = 0;  // m/s
    robotCtrl.chassisSpeed[RIGHT] = 0; // m/s
    robotCtrl.chassisTurnSpeed = 0;
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
    robotCtrl.bodyPitch = balance.fiFb / RAD_PER_DEG;
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
        robotCtrl.bodyPitch = balance.fiFb / RAD_PER_DEG;
        break;
    }
    transeOverFlag = true;
    return transeOverFlag;
}