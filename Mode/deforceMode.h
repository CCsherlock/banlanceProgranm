#ifndef _DEFORCE_MODE_H_
#define _DEFORCE_MODE_H_

#include "board.h"
#include "mode_task.h"


class DeforceMode : public RunMode
{
private:
    /* data */
public:
    DeforceMode() : RunMode(DEFORCE){};
    uint8_t intoModeRun(RobotMotion _modeLast) override;
    void inModeRun() override;
    void reset() override { transeResetFlag = true; };
};


#endif