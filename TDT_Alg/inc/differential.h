#ifndef _DIFFERENTIAL_H_
#define _DIFFERENTIAL_H_

#include "board.h"

class Differential
{
private:
    /* data */
public:
    Differential();
    float* diffNumPtr;
    float diffTime;
    float calDiffResult();
    void init(float* _diffNumPtr, int _diffTime);
    bool initFlag = false;
    float diffResult;
    uint64_t recodeTime;
    float recodeNumber,recodeNumber_last;
};

#endif