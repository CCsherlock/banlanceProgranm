#ifndef _INSTABLE_CHECH_TASK_H_
#define _INSTABLE_CHECH_TASK_H_

#include "board.h"

enum CheckState
{
    CHECK_OK = 0U,
    CHECK_WARNING,
    CHECK_ERROR
};

enum CheckList
{
    LEFT_CHASSIS_STATE = 0U,
    LEFT_LEG_STATE,
    RIGHT_LEG_STATE,
    BODY_FI_STATE,
    STATE_LIST_NUMBER
};
class RobotStateList
{
private:
    /* data */
    uint64_t instableTimeThreshold; // ms
    int64_t stateTatolTime;        // ms
    uint64_t recodeTime, recodeTime_last;
    CheckState nowState;

public:
    RobotStateList(/* args */);
    void setInstableTime(uint64_t timeThreshold);
    void lostStableRecode();  // 记录失稳时间
    void resetState();        // 重置失稳状态
    CheckState getNowState(); // 获取当前状态
    bool TrigeFlag = 0;       // 失稳触发标志位
		bool errorCntRecordFlag;
		uint16_t errorCntTimes;
};

class InstableCheck
{
private:
    /* data */
    bool listInitFlag = false;
public:
    InstableCheck(/* args */);
    RobotStateList stateList[STATE_LIST_NUMBER];
    CheckState nowState[STATE_LIST_NUMBER];
    void checkInit();
    void checkReset();
    void checkLoop();
    uint8_t getNowRobotState;
    void checkList();
};
extern InstableCheck robotStabelCheck;
extern uint8_t instableFlag;
#endif