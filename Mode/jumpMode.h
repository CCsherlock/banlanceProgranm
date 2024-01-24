#ifndef _JUMP_MODE_H_
#define _JUMP_MODE_H_

#include "board.h"
#include "mode_task.h"
class JumpMode : public RunMode
{
public:
    JumpMode() : RunMode(JUMP) {modeInit();};
    uint8_t intoModeRun(RobotMotion _modeLast) override;
    void inModeRun() override;
    void reset() override;
    void modeInit() override{};
    RampCurve thetaRamp[2];
    uint8_t recodeTranseFlag = 0;
    float thetaStart[2], thetaEnd[2];
    uint8_t jumpFinishFlag = true;
};
#endif