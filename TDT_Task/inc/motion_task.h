#ifndef _MOTION_TASK_H_
#define _MOTION_TASK_H_

#include "board.h"
#include "mode_task.h"
#include "angleTranseform.h"
class Motion
{
private:
    uint8_t modeChageFlag = false;

public:
    Motion();
    RobotMotion robotMode, robotMode_last, robotModelastRecode; // 机器人总运行模式
    void motionModeSwitch();
    void chassisSpeedCtrl();
    void bodyPitchCtrl();
    void bodyThetaCtrl();
    void runModeJudge();
    uint8_t modeChangeFlag = 0;
		uint8_t modeRecodeFlag = 0;
    float fiOffset = -4;
};

extern void motionLoop();
#endif