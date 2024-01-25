#include "slideJudge_task.h"
#include "imu_task.h"
#include "chassis_task.h"
#include "differential.h"
#include "lqrCtrl_task.h"
Differential diffXspeed;
Differential diffWspped;
Differential diffWGyro;

SlideJudge slideJude;
#define SLIDE_X_ACC_ERR_THREDHOLD 100
#define SLIDE_W_SPEED_ERR_THREDHOLD 100
#define SLIDE_X_ACC_THREDHOLD 500
#define SLIDE_W_SPEED_THREDHOLD 500
#define SLIDE_JUDGE_TIME 100 // ms
SlideJudge::SlideJudge(/* args */)
{
}

void SlideJudge::calculateAcc()
{
    bodyxAccFrommMotor = diffXspeed.calDiffResult();
    bodyxAccFromImu = bmi088Cal->acc.accValue.data[0];
    bodywSpeedFromImu = bmi088Cal->gyro.radps.data[2];
    bodywSpeedFrommMotor = balance.wSpeedFb;
}
void SlideJudge::init()
{
    diffXspeed.init(&balance.speedFb, 200);
    diffWGyro.init(&bmi088Cal->gyro.radps.data[2], 300);
    diffWspped.init(&balance.wSpeedFb, 200);
    initFlag = true;
}
void SlideJudge::calculateXSpeed()
{
    if (isSlide)
    {
        if (!xSpeedRecodeFlag)
        {
            recodeXspeed = balance.speedFb;
            recodeXAcc = bodyxAccFromImu;
            recordXTime = getSysTimeUs();
            xSpeedRecodeFlag = true;
        }
        xSpeedEstimate = recodeXspeed + bodyxAccFromImu * (timeIntervalFrom(recordXTime) / 1e6f);
    }
    else
    {
        xSpeedRecodeFlag = false;
    }
}
void SlideJudge::judgeRun()
{
    if (!initFlag)
    {
        init();
    }
    calculateAcc();
    calculateXSpeed();
    xAccErr = bodyxAccFrommMotor - bodyxAccFromImu;
    wSpeedErr = bodywSpeedFrommMotor - bodywSpeedFromImu;
    if (ABS(xAccErr) > SLIDE_X_ACC_ERR_THREDHOLD &&       // 加速度差阈值
        ABS(bodyxAccFrommMotor) > ABS(bodyxAccFromImu) && // 轮子加速度大于身体加速度
        ABS(bodyxAccFrommMotor) > SLIDE_X_ACC_THREDHOLD)  // 轮子自身加速度阈值
    {
        isXSlide = true;
    }

    if (ABS(wSpeedErr) > SLIDE_W_SPEED_ERR_THREDHOLD &&       // 加速度差阈值
        ABS(bodywSpeedFrommMotor) > ABS(bodywSpeedFromImu) && // 轮子加速度大于身体加速度
        ABS(bodywSpeedFrommMotor) > SLIDE_W_SPEED_THREDHOLD)  // 轮子自身加速度阈值
    {
        isWSlide = true;
    }

    if (isXSlide || isWSlide)
    {
        slideJudgeTime++;
    }
    else
    {
        if (slideJudgeTime == 0)
        {
            slideJudgeTime = 0;
        }
        else
        {
            slideJudgeTime--;
        }
    }
    if (slideJudgeTime >= SLIDE_JUDGE_TIME)
    {
        isSlide = true;
        slideJudgeTime = SLIDE_JUDGE_TIME;
    }
    else
    {
        isSlide = false;
    }
}