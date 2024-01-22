#ifndef _FALL_RECOVERY_TASK_H_
#define _FALL_RECOVERY_TASK_H_

#include "board.h"
#include "pid.h"
enum FallState
{
    NO_FALL = 0U,
    HALF_FALL,
    COMPLETE_FALL
};
class FallRecover
{
private:
    /* data */
public:
    FallRecover(/* args */);
    Pid *legPidOuter[2];
    Pid *legPidInner[2];
    PidParam legCtrlOuterParam[2];
    PidParam legCtrlInnerParam[2];
    void fallInit();
    bool fallInitFlag;
    void fallJudge();
    void fallCtrl();
    void fallPidCalculate();
    void fallOutput();
    void reset();
    uint8_t pidCalcnt = 0;
    bool fallJudgeFlag = false;
    float legAngleCtrl[2];
    FallState nowFallSate;
};
void fallRecoveryRun();
extern FallRecover falling;


#endif