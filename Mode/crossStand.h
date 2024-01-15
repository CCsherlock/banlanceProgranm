#ifndef _CROSS_STAND_H_
#define _CROSS_STAND_H_

#include "board.h"
#include "mode_task.h"
class CrossStandMode : public RunMode
{
public:
    CrossStandMode() : RunMode(CROSS_STAND) {};
    uint8_t intoModeRun(RobotMotion _modeLast) override;
    void inModeRun() override;
    void reset() override;
    void modeInit() override;
    RampCurve thetaRamp[2];
    uint8_t recodeTranseFlag = 0;
    float thetaStart[2], thetaEnd[2];
};
#endif