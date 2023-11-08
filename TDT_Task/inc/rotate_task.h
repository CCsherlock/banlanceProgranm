#ifndef _ROTATE_TASK_H_
#define _ROTATE_TASK_H_

#include "board.h"
#include "motor.h"
#include "pid.h"



class RotateTask
{
private:
    /* data */
public:
    RotateTask(/* args */);
    Motor *motor;
    Pid *motorPidOuter;
    PidParam *motorPidInner,*positionPid;
    void taskInit();
    void run();
    void rotateSpeedCal();
    void positionCal();
    void stateSwitch();
    int initPosition = 0;
    bool initFlag = false;
    u8 stateFlag;
    u8 rotateStar = 2;
    float speedSet;
    float positionSet;
    float position;
    float angle,angleTotal;
    u8 arriveFlag = 0;
};

extern RotateTask rotate;

#endif // !_ROTATE_TASK_H_