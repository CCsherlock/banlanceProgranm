#ifndef _SLIDE_JUDGE_TASK_H_
#define _SLIDE_JUDGE_TASK_H_

#include "board.h"

class SlideJudge
{
private:
    /* data */
public:
    SlideJudge(/* args */);
    uint8_t isSlide = 0;
    uint8_t isXSlide = 0;
    uint8_t isWSlide = 0;
    void judgeRun();
    void calculateAcc();
    void init();
    uint8_t initFlag = 0;
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
    uint8_t xSpeedRecodeFlag = 0;
};
extern SlideJudge slideJude;

#endif