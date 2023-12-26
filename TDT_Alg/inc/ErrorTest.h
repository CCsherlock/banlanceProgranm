#ifndef _ERROR_TEST_H_
#define _ERROR_TEST_H_

#include "board.h"
enum errorListAll
{
    ENCODE_BOARD_LOST = 0U,
		MotorLostError,
    AllErrorNumber
};
#define NO_ERROR 0
class ErrorList
{
private:
    /* data */
public:
    ErrorList();
    bool errorState = false;
    bool errorCheck();
		bool trigeFlag = false;
		bool error();
		bool nError();
    bool getErrorState() { return errorState; };
    float errorTimeTreshold = 10;
    float errorTimeTotal = 0;
    float errorCheckTime,errorCheckTime_last;
    void setTresholdTime(float time) { errorTimeTreshold = time; }
};

extern ErrorList errorList[AllErrorNumber];


void ErrorChechAlarm();
void errorTestList();
#endif