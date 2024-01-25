#ifndef _SLIDE_JUDGE_TASK_H_
#define _SLIDE_JUDGE_TASK_H_

#include "board.h"

class SlideJudge
{
private:
    /* data */
public:
    SlideJudge(/* args */);
    bool isSlide = false;
    bool isXSlide = false;
    bool isWSlide = false;
    void judgeRun();
    void calculateAcc();
    void init();
    bool initFlag = false;
    float bodyxAccFromImu,bodyxAccFromImu_cal;
    float bodyxAccFrommMotor;
    float bodywSpeedFromImu;
    float bodywSpeedFrommMotor;
    float xAccErr, wSpeedErr;
    float xSpeedEstimate;
    uint32_t slideJudgeTime;
    void calculateXSpeed();
    float recodeXspeed, recodeXAcc;
    uint64_t recordXTime;
    bool xSpeedRecodeFlag = false;
};
extern SlideJudge slideJude;

#endif