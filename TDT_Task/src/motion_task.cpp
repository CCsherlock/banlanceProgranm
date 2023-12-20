#include "motion_task.h"
#include "dbus.h"
#include "imu_task.h"
#include "lqrCtrl_task.h"
using namespace RCS;
Motion robotMotion;
RampCurve ThetaUpRamp;
Motion::Motion()
{
}

void Motion::motionModeSwitch()
{
    if (deforceFlag)
    {
        robotMode = DEFORCE;
        robotMode_last = robotMode;
        return;
    }
    if (RC.Key.SW2 == Mid && RC.Key.SW1 == Mid) // 上力置中
    {
        robotMode = SIT;
    }
    else if (RC.Key.SW2 == Mid && RC.Key.SW1 == Down)
    {
        robotMode = SLEEP;
    }
    else if (RC.Key.SW2 == Mid && RC.Key.SW1 == Up)
    {
        robotMode = CROSS_STAND;
    }
    robotMode_last = robotMode;
}
/**
 * @brief
 *
 */
float speedP = 10;
float turnP = 5;
void Motion::chassisSpeedCtrl()
{
    switch (robotMode)
    {
    case DEFORCE:
        robotCtrl.chassisSpeed = 0; // m/s
        break;
    case SIT:
        robotCtrl.chassisSpeed = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V*speedP; // m/s
		    robotCtrl.chassisTurnSpeed = (RC.Key.CH[0] / 660.f) * ROBOT_MAX_V*turnP; // m/s
        break;
    default:
			  robotCtrl.chassisSpeed = 0; // m/s
        break;
    }
     balance.speedSet[LEFT] = LIMIT(robotCtrl.chassisSpeed + robotCtrl.chassisTurnSpeed, -ROBOT_MAX_V*speedP, ROBOT_MAX_V*speedP);
		 balance.speedSet[RIGHT] = LIMIT(robotCtrl.chassisSpeed - robotCtrl.chassisTurnSpeed, -ROBOT_MAX_V*speedP, ROBOT_MAX_V*speedP);
}
/**
 * @brief
 *
 */
void Motion::bodyThetaCtrl()
{
    switch (robotMode)
    {
    case DEFORCE:
        robotCtrl.bodyTheta = 0; // °
				ThetaUpRamp.reset();
        break;
    case SIT:
        robotCtrl.bodyTheta = 0;                                                                            // °
        balance.angleSet[LEFT] = standThetaCal(balance.angleFb[LEFT], robotCtrl.bodyTheta) * RAD_PER_DEG;   // rad
        balance.angleSet[RIGHT] = standThetaCal(balance.angleFb[RIGHT], robotCtrl.bodyTheta) * RAD_PER_DEG; // rad
				ThetaUpRamp.reset();
        break;
    case STAND:
        robotCtrl.bodyTheta = 90;                                                                           // °
        balance.angleSet[LEFT] = standThetaCal(balance.angleFb[LEFT], robotCtrl.bodyTheta) * RAD_PER_DEG;   // rad
        balance.angleSet[RIGHT] = standThetaCal(balance.angleFb[RIGHT], robotCtrl.bodyTheta) * RAD_PER_DEG; // rad
        break;
    case CROSS_STAND:
				robotCtrl.bodyTheta = ThetaUpRamp.ramp(crossStandSpeed, 0, crossStandAngle);		// °
        balance.angleSet[LEFT] = standThetaCal(balance.angleFb[LEFT], robotCtrl.bodyTheta) * RAD_PER_DEG;    // rad
        balance.angleSet[RIGHT] = standThetaCal(balance.angleFb[RIGHT], -robotCtrl.bodyTheta) * RAD_PER_DEG; // rad
				break;
    default:
			  robotCtrl.bodyTheta = 0;                                                                            // °
        balance.angleSet[LEFT] = standThetaCal(balance.angleFb[LEFT], robotCtrl.bodyTheta) * RAD_PER_DEG;   // rad
        balance.angleSet[RIGHT] = standThetaCal(balance.angleFb[RIGHT], robotCtrl.bodyTheta) * RAD_PER_DEG; // rad
        break;
    }
}
/**
 * @brief
 *
 */
void Motion::bodyPitchCtrl()
{
    switch (robotMode)
    {
    case DEFORCE:
        robotCtrl.bodyPitch = 0; // °
        break;
    case SIT:
        robotCtrl.bodyPitch = 0; // °
    case STAND:
        robotCtrl.bodyPitch = 0; // °
        break;
    default:
        robotCtrl.bodyPitch = 0; // °
        break;
    }
    balance.fiSet = (robotCtrl.bodyPitch+fiOffset) * RAD_PER_DEG; // rad
}
/**
 * @brief 计算稳态下内圈设定值
 *
 * @param thetaNow 当前角度值 -inf-inf rad
 * @param targetTheta 目标角度 0-360°
 * @return float 根据当前角度输出的目标值 单位°
 */
float Motion::standThetaCal(float thetaNow, float targetTheta)
{
    float resultTheta;
    float cntNow = (int)(thetaNow / RAD_PER_DEG / 360.0f);
    float degreeNow = ((int)(thetaNow / RAD_PER_DEG * 1000)) % 360000 / 1000.0f;
    if (degreeNow < 0)
    {
        degreeNow = degreeNow + 360.0f; // 转到0-360°
    }
    float angleErr = degreeNow - targetTheta;
    if (angleErr >= 0)
    {
        if (angleErr < 180)
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - angleErr;
        }
        else
        {
            resultTheta = (thetaNow / RAD_PER_DEG) + (360 - angleErr);
        }
    }
    else
    {
        if (angleErr < -180.0f)
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - (360 + angleErr);
        }
        else
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - angleErr;
        }
    }
    return resultTheta;
}
/**
 * @brief （待完成）跳跃计算函数
 * 
 * @param thetaNow 
 * @param targetAngleErr 
 * @param direction 
 * @return float 
 */
//float Motion::jumpThetaCal(float thetaNow, float targetAngleErr, int direction)
//{
//    float resultTheta;
//    float cntNow = (int)(thetaNow / RAD_PER_DEG / 360.0f);
//    float degreeNow = ((int)(thetaNow / RAD_PER_DEG * 1000)) % 360000 / 1000.0f;
//    if (degreeNow < 0)
//    {
//        degreeNow = degreeNow + 360.0f; // 转到0-360°
//    }
//}

RampCurve::RampCurve(/* args */)
{
    curveResult = 0;
    curveTime = curveTime_last = 0;
    curveInit = false;
}

float RampCurve::ramp(double slop, double start, double end)
{
    if (!curveInit)
    {
        curveResult = start;
        curveTime = (float)(getSysTimeUs()/1e6f);
        curveTime_last = curveTime;
        curveInit = true;
        curveFinish = false;
        return curveResult;
    }
    curveTime = (float)(getSysTimeUs()/1e6f);
    if (signbit(slop))
    {
        if (curveResult <= end)
        {
            curveMode = END;
            curveFinish = true;
            return end;
        }
    }
    else
    {
        if (curveResult >= end)
        {
            curveMode = END;
            curveFinish = true;
            return end;
        }
    }

    double timeErr = curveTime - curveTime_last;
    curveResult = curveResult + slop * timeErr;
    curveTime_last = curveTime;
    curveMode = RUNNING;
    return curveResult;
}

void RampCurve::reset()
{
    if (curveMode != INIT)
    {
        curveResult = 0;
        curveTime = curveTime_last = 0;
        curveMode = INIT;
			  curveInit = false;
    }
}
void motionLoop()
{
    robotMotion.motionModeSwitch(); // 设置机器人整体状态
    robotMotion.chassisSpeedCtrl(); // 设置底盘速度
    robotMotion.bodyThetaCtrl();    // 设置机器人内圈角度
    robotMotion.bodyPitchCtrl();    // 设置机体俯角
}