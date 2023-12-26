#ifndef _MOTION_TASK_H_
#define _MOTION_TASK_H_

#include "board.h"
#define ROBOT_MAX_V 1
enum RobotMotion
{
    DEFORCE = 0U, // 脱力
    SLEEP,        // 卧姿
    SIT,          // 坐姿
    STAND,        // 站姿
    CROSS_STAND   // 交叉站姿
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
    float jumpThetaCal(float thetaNow, float targetAngleErr,int direction);
    float standThetaCal(float thetaNow, float targetTheta);
    struct RobotCtrlVal
    {
        /* data */
        float chassisSpeed; // 底盘直行速度设定
				float chassisTurnSpeed;// 底盘转向速度设定
        float chassisYaw;   // 底盘Yaw方向设定
        float bodyPitch;    // 机体俯仰设定
        float bodyTheta;    // 关节角度设定
    } robotCtrl;
		float crossStandAngle = 180;
		float crossStandSpeed = 90;
		float fiOffset = -4;
};

class RampCurve
{

private:
	  enum CurveMode
    {
        INIT = 0,
        RUNNING,
        END
    };
public:
    RampCurve();
    bool curveInit = false;
    float curveResult;
    float curveTime,curveTime_last;
    bool curveFinish;
    float ramp(double slop,double start,double end);
    void reset();
    uint8_t curveMode;
};
extern void motionLoop();
#endif