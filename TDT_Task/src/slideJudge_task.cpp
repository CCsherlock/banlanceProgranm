#include "slideJudge_task.h"
#include "imu_task.h"
#include "chassis_task.h"
#include "differential.h"
#include "lqrCtrl_task.h"
#include "filter.h"
Differential diffXspeed;
Differential diffWspped;
Differential diffWGyro;

SlideJudge slideJude;

Lpf2p diffXMotorFilter;
Lpf2p diffXImuFilter;
#define SLIDE_X_ACC_ERR_THREDHOLD 0.5
#define SLIDE_W_SPEED_ERR_THREDHOLD 6
#define SLIDE_X_ACC_THREDHOLD 0.5
#define SLIDE_W_SPEED_THREDHOLD 12
#define SLIDE_W_SPEED_BODY_THREDHOLD 1
#define SLIDE_JUDGE_TIME 10 // ms
SlideJudge::SlideJudge(/* args */)
{
}

void SlideJudge::calculateAcc()
{
//    bodyxAccFrommMotor = diffXspeed.calDiffResult();
		bodyxAccFrommMotor = diffXMotorFilter.Apply(diffXspeed.calDiffResult()) * OUTER_WHEEL_RADIO;
//    bodyxAccFromImu = bmi088Cal->acc.accValue.data[0];
		bodyxAccFromImu_cal = bmi088Cal->acc.accValue.data[0] * cos(bmi088Cal->Angle.pitch*RAD_PER_DEG) + bmi088Cal->acc.accValue.data[2] * sin(bmi088Cal->Angle.pitch*RAD_PER_DEG);
		bodyxAccFromImu = diffXImuFilter.Apply(bodyxAccFromImu_cal);
    bodywSpeedFromImu = bmi088Cal->gyro.radps.data[2];
    bodywSpeedFrommMotor = balance.wSpeedFb * OUTER_WHEEL_RADIO / 2;
}
void SlideJudge::init()
{
    diffXspeed.init(&balance.speedFb, 200);
    diffWGyro.init(&bmi088Cal->gyro.radps.data[2], 300);
    diffWspped.init(&balance.wSpeedFb, 200);
		diffXMotorFilter.SetCutoffFreq(2000,10);
		diffXImuFilter.SetCutoffFreq(2000,10);
    initFlag = true;
}
void SlideJudge::calculateXSpeed()
{
    if (isSlide)
    {
        if (!xSpeedRecodeFlag)
        {
            recodeXspeed = balance.chassis->chassisXSpeed;
            recodeXAcc = bodyxAccFromImu;
            recordXTime = getSysTimeUs();
            xSpeedRecodeFlag = true;
        }
        xSpeedEstimate = recodeXspeed + bodyxAccFromImu * (timeIntervalFrom(recordXTime) / 1e6f);
				recordXTime = getSysTimeUs();
    }
    else
    {
				xSpeedEstimate = balance.chassis->chassisXSpeed;
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
        ABS(bodyxAccFrommMotor) > SLIDE_X_ACC_THREDHOLD &&	// 轮子自身加速度阈值
				ABS(bodywSpeedFromImu) < SLIDE_W_SPEED_BODY_THREDHOLD) 
    {
        isXSlide = 1;
    }
		else
		{
				isXSlide = 0;
		}

    if (ABS(wSpeedErr) > SLIDE_W_SPEED_ERR_THREDHOLD &&       // 加速度差阈值
        ABS(bodywSpeedFrommMotor) > ABS(bodywSpeedFromImu) && // 轮子加速度大于身体加速度
        ABS(bodywSpeedFrommMotor) > SLIDE_W_SPEED_THREDHOLD)  // 轮子自身加速度阈值
    {
        isWSlide = 1;
    }
		else
		{
				isWSlide = 0;
		}

    if (isXSlide || isWSlide)
    {
        slideJudgeTime++;
    }
    else
    {
        if (slideJudgeTime > 0)
        {
             slideJudgeTime--;
        }
    }
    if (slideJudgeTime >= SLIDE_JUDGE_TIME)
    {
        isSlide = 1;
				balance.chasTorKpByMotion = 0;
        slideJudgeTime = SLIDE_JUDGE_TIME;
    }
    else
    {
				balance.chasTorKpByMotion = 1;
        isSlide = 0;
    }
}