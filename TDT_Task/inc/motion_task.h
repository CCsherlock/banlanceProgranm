#ifndef _MOTION_TASK_H_
#define _MOTION_TASK_H_

#include "board.h"
#define ROBOT_MAX_V 4
enum RobotMotion
{
    DEFORCE = 0U, // 脱力
    SLEEP,        // 卧姿
    SIT,          // 坐姿
    STAND         // 站姿
};
class Motion
{
private:
public:
    Motion();
    RobotMotion robotMode, robotMode_last; // 机器人总允许模式
    void motionModeSwitch();
    void chassisSpeedCtrl();
    void bodyPitchCtrl();
    void bodyThetaCtrl();
    struct RobotCtrlVal
    {
        /* data */
        float chassisSpeed; // 底盘速度设定
        float chassisYaw;   // 底盘Yaw方向设定
        float bodyPitch;    // 机体俯仰设定
        float bodyTheta;    // 关节角度设定
    } robotCtrl;
};
#endif