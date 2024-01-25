#include "jumpMode.h"

uint8_t JumpMode::intoModeRun(RobotMotion _modeLast)
{
  switch (_modeLast)
  {
  case SIT:
		 balance.roboLqr->setNowParam(balance.roboLqr->UP_PARAM); // 设置当前LQR参数方案
    thetaStart[LEFT] = balance.angleFb[LEFT];
    thetaStart[RIGHT] = balance.angleFb[RIGHT];
    robotCtrl.chassisYaw = balance.yawFb;
    robotCtrl.chassisSpeed = 0; // m/s
    thetaEnd[LEFT] = jumpThetaCal(balance.angleFb[LEFT], 360, 360, -1) * RAD_PER_DEG;
    thetaEnd[RIGHT] = jumpThetaCal(balance.angleFb[RIGHT], 360, 360, -1) * RAD_PER_DEG;
	  robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
    jumpFinishFlag = 0;
    transeOverFlag = 1;
    break;
  default:
    robotCtrl.chassisSpeed = 0; // m/s
    robotCtrl.chassisYaw = balance.yawFb;
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
  if (!jumpFinishFlag)
  {
		 balance.roboLqr->setNowParam(balance.roboLqr->UP_PARAM); // 设置当前LQR参数方案
    robotCtrl.chassisSpeed = 3; // m/s
    robotCtrl.chassisYaw = balance.yawFb;
    robotCtrl.bodyPitch = 0;
    robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT]) / 0.1, thetaStart[LEFT], thetaEnd[LEFT]);
    robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT]) / 0.1, thetaStart[RIGHT], thetaEnd[RIGHT]);
    if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish && ABS(balance.angleFb[LEFT] - robotCtrl.bodyTheta[LEFT]) < 0.1 && ABS(balance.angleFb[RIGHT] - robotCtrl.bodyTheta[RIGHT]) < 0.1)
    {
      jumpFinishFlag = 1;
    }
  }
  else
  {
		balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM); // 设置当前LQR参数方案
    robotCtrl.chassisYaw += (RC.Key.CH[0] / 660.f) * ROBOT_MAX_W;                        // rad
    robotCtrl.chassisSpeed = -(RC.Key.CH[3] / 660.f) * ROBOT_MAX_V / 2;                  // m/s
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
  thetaRamp[LEFT].reset();
  thetaRamp[RIGHT].reset();
  transeOverFlag = false;
  recodeTranseFlag = false;
  transeResetFlag = true;
}