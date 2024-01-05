#include "mode_task.h"
#include "lqrCtrl_task.h"
#include "deforceMode.h"
#include "sitMode.h"
#include "crossStand.h"
#include "jumpMode.h"
DeforceMode deforceMode;
SitMode sitMode;
CrossStandMode standMode;
JumpMode jumpMode;
int RunMode::modeNum = 0;
RunMode **RunMode::modeList = 0;
RunMode::RunMode(RobotMotion _thisMode)
{
    if (modeList == 0)
    {
        modeList = (RunMode **)malloc(sizeof(RunMode *));
    }
    else
    {
        modeList = (RunMode **)realloc(modeList, sizeof(RunMode *) * ALL_MODE_NUM);;
    }
    modeList[_thisMode] = this;
    thisMode = _thisMode;
    modeNum += 1;
}