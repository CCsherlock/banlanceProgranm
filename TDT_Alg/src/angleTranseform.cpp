#include "angleTranseform.h"
/**
 * @brief 计算稳态下内圈设定值
 *
 * @param thetaNow 当前角度值 -inf-inf rad
 * @param targetTheta 目标角度 0-360°
 * @return float 根据当前角度输出的目标值 单位°
 */
float standThetaCal(float thetaNow, float targetTheta)
{
    float resultTheta;
    float cntNow = (int)(thetaNow / RAD_PER_DEG / 360.0f);
    float degreeNow = ((int)(thetaNow / RAD_PER_DEG * 1000)) % 360000 / 1000.0f;
    if (degreeNow < 0)
    {
        degreeNow = degreeNow + 360.0f; // 转到0-360°
    }
    float angleErr = degreeNow - targetTheta;
    if (angleErr >= 0)
    {
        if (angleErr < 180)
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - angleErr;
        }
        else
        {
            resultTheta = (thetaNow / RAD_PER_DEG) + (360 - angleErr);
        }
    }
    else
    {
        if (angleErr < -180.0f)
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - (360 + angleErr);
        }
        else
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - angleErr;
        }
    }
    return resultTheta;
}

/**
 * @brief
 *
 * @param thetaNow
 * @param targetTheta
 * @return float
 */
float jumpThetaCal(float thetaNow, float targetTheta, float jumpAngle, int8_t dirction)
{
    float resultTheta;
    float cntNow = (int)(thetaNow / RAD_PER_DEG / 360.0f);
    float degreeNow = ((int)(thetaNow / RAD_PER_DEG * 1000)) % 360000 / 1000.0f;
    if (degreeNow < 0)
    {
        degreeNow = degreeNow + 360.0f; // 转到0-360°
    }
    float angleJump = degreeNow + dirction * jumpAngle;
    if (angleJump > 360)
    {
        angleJump = angleJump - 360;
    }
    else if (angleJump < 0)
    {
        angleJump = angleJump + 360;
    }
    float angleErr = angleJump - targetTheta;
    if (angleErr >= 0)
    {
        if (angleErr < 180)
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - angleErr + dirction * jumpAngle;
        }
        else
        {
            resultTheta = (thetaNow / RAD_PER_DEG) + (360 - angleErr) + dirction * jumpAngle;
        }
    }
    else
    {
        if (angleErr < -180.0f)
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - (360 + angleErr) + dirction * jumpAngle;
        }
        else
        {
            resultTheta = (thetaNow / RAD_PER_DEG) - angleErr + dirction * jumpAngle;
        }
    }
    return resultTheta;
}
