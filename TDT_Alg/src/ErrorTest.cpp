#include "ErrorTest.h"
#include "beep.h"
#include "chassis_task.h"
ErrorList errorList[AllErrorNumber];
ErrorList::ErrorList()
{

}
bool ErrorList::errorCheck()
{
    errorCheckTime = (float)(getSysTimeUs() / 1e3f); //Ms
    if (errorCheckTime - errorCheckTime_last > errorTimeTreshold * 2)
    {
        errorTimeTotal = 0;
        errorCheckTime_last = errorCheckTime;
        errorState = false;
			  trigeFlag = false;
        return errorState; // 两次检测时间超过了时间阈值的两倍
    }
		if(trigeFlag)
		{
			errorTimeTotal += errorCheckTime - errorCheckTime_last; // 连续累加出错时间
		}
    else
		{
			errorTimeTotal -= errorCheckTime - errorCheckTime_last; // 连续累加出错时间
			if(errorTimeTotal < 0)
			{
				errorTimeTotal = 0;
			}
		}
    errorCheckTime_last = errorCheckTime;
    if (errorTimeTotal > errorTimeTreshold) // 错误累加时间超过设定阈值
    {
			errorTimeTotal = errorTimeTreshold + 1;
        errorState = true;
        return errorState;
    }
    else
    {
        errorState = false;
        return errorState;
    }
}
bool ErrorList::error()
{
	return trigeFlag = true;
}
bool ErrorList::nError()
{
	return trigeFlag = false;
}
uint8_t allErrorNumber;
uint8_t priorityError = AllErrorNumber;
uint8_t errorListInit = false;
void ErrorChechAlarm()
{
	if(!errorListInit)
	{
		errorList[MotorLostError].setTresholdTime(10);
		errorList[ENCODE_BOARD_LOST].setTresholdTime(10);
		errorListInit = true;
		return;
	}
		errorTestList();
    for (uint8_t i = 0; i < AllErrorNumber; i++)
    {
				errorList[i].errorCheck();
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

void errorTestList()
{
#if defined SMALL_MODEL
	if(chssisMotor[LEFT]->canInfo.lostFlag 
		||chssisMotor[RIGHT]->canInfo.lostFlag
		||legMotor[LEFT]->canInfo.lostFlag
		||legMotor[RIGHT]->canInfo.lostFlag)
	{
		errorList[MotorLostError].error();
	}
	else
	{
		errorList[MotorLostError].nError();
	}
#else
	if(chssisMotor[LEFT]->motorInfo.lostFlag 
		||chssisMotor[RIGHT]->motorInfo.lostFlag
		||legMotor[LEFT]->motorInfo.lostFlag
		||legMotor[RIGHT]->motorInfo.lostFlag)
	{
		errorList[MotorLostError].error();
	}
	else
	{
		errorList[MotorLostError].nError();
	}
	
//	if(legMotor[LEFT]->megTrans.lostFlag||legMotor[RIGHT]->megTrans.lostFlag)
//	{
//		errorList[ENCODE_BOARD_LOST].error();
//	}
//	else
//	{
//		errorList[ENCODE_BOARD_LOST].nError();
//	}
#endif
	
}