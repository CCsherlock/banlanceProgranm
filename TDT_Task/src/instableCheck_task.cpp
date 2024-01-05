#include "instableCheck_task.h"
#include "chassis_task.h"
#include "lqrCtrl_task.h"
#include "motion_task.h"
/*各项检测阈值*/
#define CHASSIS_SPEED_THRESHOLD 40
#define LEG_SPEED_THRESHOLD 20
#define BODY_FI_THRESHOLD 70 * RAD_PER_DEG
uint8_t instableFlag = false;
RobotStateList::RobotStateList(/* args */)
{
}
/**
 * @brief 设置失稳时间阈值
 *
 * @param timeThreshold 单位 Ms
 */
void RobotStateList::setInstableTime(uint64_t timeThreshold)
{
    instableTimeThreshold = timeThreshold * 1000;
}
void RobotStateList::lostStableRecode()
{
    if (timeIntervalFrom(recodeTime_last) > 2 * instableTimeThreshold)
    {
        resetState();
        return;
    }
    if (TrigeFlag == true)
    {
        stateTatolTime += timeIntervalFrom(recodeTime_last);
    }
    else
    {
        stateTatolTime -= timeIntervalFrom(recodeTime_last);
        if (stateTatolTime < 0)
        {
            stateTatolTime = 0;
        }
    }
    recodeTime_last = getSysTimeUs();
    if (stateTatolTime > instableTimeThreshold)
    {
        stateTatolTime = instableTimeThreshold + 1;
        nowState = CHECK_ERROR;
        if (!errorCntRecordFlag)
        {
            errorCntTimes++;
            errorCntRecordFlag = true;
        }
    }
    else if (stateTatolTime > instableTimeThreshold / 2)
    {
        nowState = CHECK_WARNING;
        errorCntRecordFlag = false;
    }
    else
    {
        nowState = CHECK_OK;
        errorCntRecordFlag = false;
    }
}
void RobotStateList::resetState()
{
    stateTatolTime = 0;
    errorCntRecordFlag = 0;
    errorCntTimes = 0;
    recodeTime_last = recodeTime = getSysTimeUs();
    nowState = CHECK_OK;
}
CheckState RobotStateList::getNowState()
{
    return nowState;
}
InstableCheck::InstableCheck(/* args */)
{
}
InstableCheck robotStabelCheck;
void InstableCheck::checkInit()
{
    for (uint8_t i = 0; i < STATE_LIST_NUMBER; i++)
    {
        /* code */
        stateList[i].setInstableTime(10);
        stateList[i].resetState();
    }
}
void InstableCheck::checkLoop()
{
    uint8_t errCnt = 0;
    if (!listInitFlag)
    {
        checkInit();
        listInitFlag = true;
    }
    checkList();
    for (uint8_t i = 0; i < STATE_LIST_NUMBER; i++)
    {
        /* code */
        stateList[i].lostStableRecode();
        nowState[i] = stateList[i].getNowState();
        if (nowState[i] == CHECK_ERROR)
        {
            errCnt++;
        }
    }
    if (errCnt)
    {
        instableFlag = true;
    }
}
void InstableCheck::checkReset()
{
    instableFlag = false;
    for (uint8_t i = 0; i < STATE_LIST_NUMBER; i++)
    {
        stateList[i].resetState();
        nowState[i] = stateList[i].getNowState();
    }
}
void InstableCheck::checkList()
{
    /*左轮速度过快*/
    if (ABS(balance.speedFb[LEFT]) > CHASSIS_SPEED_THRESHOLD)
    {
        stateList[LEFT_CHASSIS_STATE].TrigeFlag = true;
    }
    else
    {
        stateList[LEFT_CHASSIS_STATE].TrigeFlag = false;
    }
    /*右轮轮速过快*/
    if (ABS(balance.speedFb[RIGHT]) > CHASSIS_SPEED_THRESHOLD)
    {
        stateList[RIGHT_CHASSIS_STATE].TrigeFlag = true;
    }
    else
    {
        stateList[RIGHT_CHASSIS_STATE].TrigeFlag = false;
    }
    /*左腿摆速过快*/
    if (ABS(balance.angleSpeedFb[LEFT]) > LEG_SPEED_THRESHOLD)
    {
        stateList[LEFT_LEG_STATE].TrigeFlag = true;
    }
    else
    {
        stateList[LEFT_LEG_STATE].TrigeFlag = false;
    }
    /*右腿摆速过快*/
    if (ABS(balance.angleSpeedFb[LEFT]) > LEG_SPEED_THRESHOLD)
    {
        stateList[LEFT_LEG_STATE].TrigeFlag = true;
    }
    else
    {
        stateList[LEFT_LEG_STATE].TrigeFlag = false;
    }
    /*机体倾角*/
    if (ABS(balance.fiFb) > BODY_FI_THRESHOLD && robotMotion.robotMode_last!=DEFORCE)
    {
        stateList[BODY_FI_STATE].TrigeFlag = true;
    }
    else
    {
        stateList[BODY_FI_STATE].TrigeFlag = false;
    }
}