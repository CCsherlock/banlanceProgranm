#ifndef _SIT_MODE_H_
#define _SIT_MODE_H_

#include "board.h"
#include "mode_task.h"

class SitMode : public RunMode
{
private:
    /* data */
public:
    SitMode(/* args */) : RunMode(SIT) {};
    uint8_t intoModeRun(RobotMotion _modeLast) override;
    void inModeRun() override;
    void reset() override;
    void modeInit() override;
    RampCurve pitchRamp;
    RampCurve thetaRamp[2];
		Pid *speedPid[2];
		PidParam speedParam;
    uint8_t recodeTranseFlag = 0;
    float thetaStart[2], thetaEnd[2];
    float fiStart, fiEnd;
};


#endif