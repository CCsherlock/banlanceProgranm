#include "sitMode.h"
float sitSpeedP = -10;
/**
 * @brief 进入坐姿模式切换状态控制
 *
 * @param _modeLast 上一个状态量
 * @return uint8_t
 */
uint8_t SitMode::intoModeRun(RobotMotion _modeLast)
{
    switch (_modeLast)
    {
    case SIT:                                                      // 从坐姿切换到坐姿（存在于在坐姿切换至别的状态但是没有完全完成的情况）
        balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM); // 设置当前LQR参数方案
        robotCtrl.chassisSpeed = 0;                                // m/s
        robotCtrl.chassisYaw = balance.yawFb;                      // rad
        robotCtrl.bodyPitch = 0;                                   // rad
        if (!recodeTranseFlag)                                     // 第一次进入进行初始位置记录与计算
        {
            thetaStart[LEFT] = balance.angleFb[LEFT];   // 记录当前起始角度
            thetaStart[RIGHT] = balance.angleFb[RIGHT]; // 记录当前起始角度
            thetaRamp[LEFT].reset();                    // 转换曲线重置
            thetaRamp[RIGHT].reset();                   // 转换曲线重置
            fiStart = balance.fiFb;                     // 记录当前起始角度
            fiEnd = fiStart;
            pitchRamp.reset();
            recodeTranseFlag = 1; // 位置记录标志位
        }
        /*进行角度转换曲线计算*/
        robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT]), thetaStart[LEFT], thetaEnd[LEFT]);
        robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT]), thetaStart[RIGHT], thetaEnd[RIGHT]);
        if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish)
        {
            thetaRamp[LEFT].reset();
            thetaRamp[RIGHT].reset();
            transeOverFlag = true;
        }
        break;
    case DEFORCE:                                                  // 从脱力模式进入坐姿
        balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM); // 设置当前LQR参数方案
        robotCtrl.chassisSpeed = 0;                                // m/s
        robotCtrl.chassisYaw = balance.yawFb;                      // rad
        if (!recodeTranseFlag)                                     // 第一次进入进行初始位置记录与计算
        {
            thetaStart[LEFT] = balance.angleFb[LEFT];   // 记录当前起始角度
            thetaStart[RIGHT] = balance.angleFb[RIGHT]; // 记录当前起始角度
            thetaRamp[LEFT].reset();                    // 转换曲线重置
            thetaRamp[RIGHT].reset();                   // 转换曲线重置
            fiStart = balance.fiFb;                     // 记录当前起始角度
            fiEnd = 0;                                  // 设置起身后的角度 rad
            pitchRamp.reset();                          // 转换曲线重置
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
        robotCtrl.bodyPitch = pitchRamp.ramp((fiEnd - fiStart) * 2, fiStart, fiEnd); // 起身Pitch轴角度计算，0.5s完成
        robotCtrl.chaTorqueKp = 0;                                                   // 在未完成起身前，不输出外圈力矩
				robotCtrl.legTorqueKp = 1;																									 // 在未完成起身前，输出腿部力矩
        if (pitchRamp.curveFinish && (ABS(balance.fiFb - fiEnd) < (30 * RAD_PER_DEG)))
        {
            pitchRamp.reset(); // 转换曲线重置
            transeOverFlag = true;
            robotCtrl.chaTorqueKp = 1; // 恢复输出外圈力矩
        }
        break;
    case CROSS_STAND:                                              // 从交错站立模式恢复坐姿
        balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM); // 设置当前LQR参数方案
        robotCtrl.chassisSpeed = 0;                                // m/s
        robotCtrl.chassisYaw = balance.yawFb;
        robotCtrl.bodyPitch = 0;
        if (!recodeTranseFlag) // 第一次进入进行初始位置记录与计算
        {
            thetaEnd[LEFT] = jumpThetaCal(balance.angleFb[LEFT], 0, 180, -1) * RAD_PER_DEG;  // 设置恢复角度rad
            thetaEnd[RIGHT] = jumpThetaCal(balance.angleFb[RIGHT], 0, 180, 1) * RAD_PER_DEG; // 设置恢复角度rad
            thetaStart[LEFT] = balance.angleFb[LEFT];                                        // 记录当前起始角度
            thetaStart[RIGHT] = balance.angleFb[RIGHT];                                      // 记录当前起始角度
            thetaRamp[LEFT].reset();                                                         // 转换曲线重置
            thetaRamp[RIGHT].reset();                                                        // 转换曲线重置
            recodeTranseFlag = 1;
        }
        robotCtrl.bodyTheta[LEFT] = thetaRamp[LEFT].ramp((thetaEnd[LEFT] - thetaStart[LEFT]) * 2, thetaStart[LEFT], thetaEnd[LEFT]);
        robotCtrl.bodyTheta[RIGHT] = thetaRamp[RIGHT].ramp((thetaEnd[RIGHT] - thetaStart[RIGHT]) * 2, thetaStart[RIGHT], thetaEnd[RIGHT]);
        if (thetaRamp[LEFT].curveFinish && thetaRamp[RIGHT].curveFinish && ABS(balance.angleFb[LEFT] - thetaEnd[LEFT]) < 0.8 && ABS(balance.angleFb[RIGHT] - thetaEnd[RIGHT]) < 0.8) // 当满足条件后进入坐姿模式
        {
            thetaRamp[LEFT].reset();  // 转换曲线重置
            thetaRamp[RIGHT].reset(); // 转换曲线重置
            transeOverFlag = true;
        }
        break;
    case JUMP:                                                     // 从跳跃模式切换回坐姿
        balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM); // 设置当前LQR参数方案
        robotCtrl.chassisSpeed = 0;                                // m/s
        robotCtrl.chassisYaw = balance.yawFb;
        robotCtrl.bodyPitch = 0;
        robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
        robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
				robotCtrl.chaTorqueKp = 0; //跳跃恢复坐姿脱力保护
				robotCtrl.legTorqueKp = 0;//跳跃恢复坐姿脱力保护
        transeOverFlag = true;
        break;
    default:
        transeOverFlag = false;
        break;
    }
    return transeOverFlag;
}
/**
 * @brief 坐姿控制模式
 *
 */
void SitMode::inModeRun()
{
    balance.roboLqr->setNowParam(balance.roboLqr->DOWN_PARAM); // 设置当前LQR参数方案
#if defined START_FROM_UP
    robotCtrl.chassisYaw += (RC.Key.CH[0] / 660.f) * ROBOT_MAX_W; // m/s
    // robotCtrl.chassisYaw = balance.yawFb;
    // robotCtrl.chassisTurnSpeed = (RC.Key.CH[0] / 660.f) * ROBOT_MAX_W * 10;
    robotCtrl.chassisSpeed = (RC.Key.CH[3] / 660.f) * ROBOT_MAX_V / 10;                    // m/s
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 180) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 180) * RAD_PER_DEG; // rad
#endif

#if defined START_FROM_DOWN
    robotCtrl.chassisYaw += (RC.Key.CH[0] / 660.f) * ROBOT_MAX_W;                        // rad
    robotCtrl.chassisSpeed = -(RC.Key.CH[3] / 660.f) * ROBOT_MAX_V / 2;                  // m/s
    robotCtrl.bodyTheta[LEFT] = standThetaCal(balance.angleFb[LEFT], 0) * RAD_PER_DEG;   // rad
    robotCtrl.bodyTheta[RIGHT] = standThetaCal(balance.angleFb[RIGHT], 0) * RAD_PER_DEG; // rad
#endif
    robotCtrl.bodyPitch = 0;
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = false;
}
/**
 * @brief 坐姿重置函数
 * 
 */
void SitMode::reset()
{
    thetaRamp[LEFT].reset();
    thetaRamp[RIGHT].reset();
    pitchRamp.reset();
    transeOverFlag = false;
    recodeTranseFlag = false;
    transeResetFlag = true;
}
