#include "ErrorTest.h"
#include "beep.h"
ErrorList errorList[AllErrorNumber];
ErrorList::ErrorList()
{

}
bool ErrorList::errorCheck()
{
    errorCheckTime = (float)(getSysTimeUs() / 1e3f);
    if (errorCheckTime - errorCheckTime_last > errorTimeTreshold * 2)
    {
        errorTimeTotal = 0;
        errorCheckTime_last = errorCheckTime;
        errorState = false;
        return errorState; // 两次检测时间超过了时间阈值的两倍
    }
    else
    {
        errorTimeTotal += errorCheckTime - errorCheckTime_last; // 连续累加出错时间
    }
    errorCheckTime_last = errorCheckTime;
    if (errorTimeTotal > errorTimeTreshold) // 错误累加时间超过设定阈值
    {
        errorState = true;
        return errorState;
    }
    else
    {
        errorState = false;
        return errorState;
    }
}
uint8_t allErrorNumber;
uint8_t priorityError = AllErrorNumber;
void ErrorChechAlarm()
{
    for (uint8_t i = 0; i < AllErrorNumber; i++)
    {
        if(errorList[i].getErrorState())
        {
            if(i < priorityError)
            {
                priorityError = i;
            }
            allErrorNumber++;
        }   
    }
    if(allErrorNumber!=0)
    {
        if(allErrorNumber>3)
        {
            beep.setBeepTone(HIGH_TONE);
        }
        else
        {
            beep.setBeepTone(LOW_TONE);
        }
        beep.beepAlarm(priorityError+1);
    }
    else
    {
        beep.beepAlarm(NO_ERROR);
    }
		 allErrorNumber = 0;
}