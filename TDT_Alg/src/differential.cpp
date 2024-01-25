#include "differential.h"
Differential::Differential()
{
    recodeNumber = 0;
    recodeNumber_last = 0;
    recodeTime = 0;
}
float Differential::calDiffResult()
{
    if (!initFlag)
    {
        return 0;
    }
		recodeNumber = *diffNumPtr;
    if ((timeIntervalFrom(recodeTime) / 1e6f) > diffTime)
    {
        diffResult = (recodeNumber - recodeNumber_last) / (timeIntervalFrom(recodeTime) / 1e6f);
        recodeTime = getSysTimeUs();
        recodeNumber_last = recodeNumber;
    }
    return diffResult;
}
void Differential::init(float *_diffNumPtr, int _diffTime)
{
    if (_diffNumPtr != nullptr)
    {
        diffNumPtr = _diffNumPtr;
        recodeNumber = *diffNumPtr;
        recodeNumber_last = recodeNumber;
        recodeTime = getSysTimeUs();
        diffTime = (1.0f / (float)_diffTime);
        initFlag = true;
    }
}