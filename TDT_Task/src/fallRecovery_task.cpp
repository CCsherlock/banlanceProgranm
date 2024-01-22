#include "fallRecovery_task.h"
#include "chassis_task.h"
#include "dbus.h"
#include "imu_task.h"
#define FALL_MAX_T 1
#define FALL_MAX_V PI
FallRecover falling;
FallRecover::FallRecover(/* args */)
{
}
void FallRecover::fallInit()
{
    for (uint8_t i = 0; i < 2; i++)
    {
        /* code */
        legPidOuter[i] = new Pid(1);
        legPidOuter[i]->fbValuePtr[0] = &legMotor[i]->megAngle; // rad
        legPidOuter[i]->paramPtr = &legCtrlOuterParam[i];

        legCtrlOuterParam[i].kp = 0;
        legCtrlOuterParam[i].ki = 0;
        legCtrlOuterParam[i].integralErrorMax = 0;
        legCtrlOuterParam[i].resultMax = FALL_MAX_V;
        legCtrlOuterParam[i].positiveFBFlag = 0;

        legPidInner[i] = new Pid(1);
        legPidInner[i]->fbValuePtr[0] = &legMotor[i]->motorInfo.motor_fdb.speed; // radps
        legPidInner[i]->paramPtr = &legCtrlInnerParam[i];

        legCtrlInnerParam[i].kp = 0;
        legCtrlInnerParam[i].ki = 0;
        legCtrlInnerParam[i].integralErrorMax = 0;
        legCtrlInnerParam[i].resultMax = FALL_MAX_T;
        legCtrlInnerParam[i].positiveFBFlag = 0;
    }
}
void FallRecover::reset()
{
    for (uint8_t i = 0; i < 2; i++)
    {
        /* code */
        legPidOuter[i]->Clear();
        legPidInner[i]->Clear();
    }
    
}
void FallRecover::fallCtrl()
{
    if (deforceFlag)
    {
        legAngleCtrl[LEFT] = legMotor[LEFT]->megAngle;
        legAngleCtrl[RIGHT] = legMotor[RIGHT]->megAngle;
    }
    else if(RC.Key.CH[4] == 2)
    {
        legAngleCtrl[LEFT] += (RC.Key.CH[0] / 660.0f) * 0.01;
        legAngleCtrl[RIGHT] += (RC.Key.CH[2] / 660.0f) * 0.01;
    }
		else
		{
        legAngleCtrl[LEFT] = legMotor[LEFT]->megAngle;
        legAngleCtrl[RIGHT] = legMotor[RIGHT]->megAngle;		
		}
}
void FallRecover::fallPidCalculate()
{
    fallCtrl();
    if (pidCalcnt >= 2)
    {
        for (uint8_t i = 0; i < 2; i++)
        {
            legPidOuter[i]->Calculate(0);
        }
				pidCalcnt = 0;
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        legPidInner[i]->Calculate(legPidOuter[i]->result);
    }
		pidCalcnt++;
}
float legPidOutputKp = 0;
void FallRecover::fallOutput()
{
    if (!deforceFlag)
    {
        for (uint8_t i = 0; i < 2; i++)
        {
            if (legMotor[i]->motorInfo.motor_mode != RUN_MODE)
            {
                for (uint8_t j = 0; j < 50; j++)
                {
                    legMotor[i]->enableMotor();
                }
            }
            legMotor[i]->motorCtrlMode(legPidInner[i]->result * legPidOutputKp, 0, 0, 0, 0);
        }
    }
    else
    {
        for (uint8_t i = 0; i < 2; i++)
        {
            /* code */
            legMotor[i]->stopMotor(0);
        }
    }
}
void FallRecover::fallJudge()
{
    if (ABS(bmi088Cal->Angle.roll) > 50 && ABS(bmi088Cal->Angle.pitch) < 40)
    {
        nowFallSate = COMPLETE_FALL;
    }
    else if (ABS(bmi088Cal->Angle.roll) > 30 && ABS(bmi088Cal->Angle.pitch) < 40)
    {
        nowFallSate = HALF_FALL;
    }
    else
    {
        nowFallSate = NO_FALL;
    }
}
void fallRecoveryRun()
{
    if (!falling.fallInitFlag)
    {
        falling.fallInit();
        falling.fallInitFlag = 1;
        return;
    }
    falling.fallJudge();
    if (falling.nowFallSate == COMPLETE_FALL)
    {
        falling.fallPidCalculate();
        falling.fallOutput();
    }
}