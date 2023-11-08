#ifndef _LQRCTRL_TASK_H_
#define _LQRCTRL_TASK_H_

#include "board.h"
#include "lqr.h"
#include "chassis_task.h"
class LqrCtrl
{
private:
    /* data */
public:
    LqrCtrl(/* args */);
    Lqr *roboLqr;
    Chassis *chassis;
    void LqrInit();
    bool lqrInitflag = false;
    float setValue[10];
    float fbValue[10];
    void getAllFbValue();
    void getAllSetValue();
    void lqrCalRun();
    void lqrOutput();
    void getSpeedFb();
    void getThetaFb();
    void getFiFb();
    void lqrKset();
    bool lqrKsetFlag = false;
    float xSet[2] = {0, 0};
    float xFb[2] = {0, 0};
    float speedSet[2] = {0, 0};
    float speedFb[2] = {0, 0};
    float angleSet[2] = {0, 0};
    float angleFb[2] = {0, 0};
    float angleFb_last[2] = {0,0};
    float angleSpeedSet[2] = {0, 0};
    float angleSpeedFb[2] = {0, 0};
    float fiSet;
    float fiSpeedSet;
    float fiFb,fiFb_last;
    float fiSpeedFb;
};
extern float laqK_buffer[40];
void linkLqrFlash();
void saveLqrMessage();
void lqrRunTask();
extern LqrCtrl balance;
#endif