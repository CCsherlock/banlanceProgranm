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
#define SLIDE_X_ACC_ERR_THREDHOLD 1   // x方向加速度差值阈值
#define SLIDE_X_SPEED_ERR_THREDHOLD 0.001 // x方向速度差值阈值
#define SLIDE_W_SPEED_ERR_THREDHOLD 6   // w方向速度差值阈值
#define SLIDE_X_ACC_THREDHOLD 1       // x方向加速度判定阈值
#define SLIDE_X_SPEED_THREDHOLD 0.05       // x方向加速度判定阈值
#define SLIDE_W_SPEED_THREDHOLD 12      // w方向轮子计算速度判定阈值
#define SLIDE_W_SPEED_BODY_THREDHOLD 1  // w方向陀螺仪判定阈值,给X方向判定使用
#define SLIDE_JUDGE_TIME 50             // 打滑判定时间裕度ms
SlideJudge::SlideJudge(/* args */)
{
}

void SlideJudge::calculateAcc()
{
    //    bodyxAccFrommMotor = diffXspeed.calDiffResult();
    bodyxAccFrommMotor = diffXMotorFilter.Apply(diffXspeed.calDiffResult()) * OUTER_WHEEL_RADIO; // 加速度低通滤波
    //    bodyxAccFromImu = bmi088Cal->acc.accValue.data[0];
    /*将z轴加速度和x轴加速度合成到水平移动方向上*/
    bodyxAccFromImu_cal = bmi088Cal->acc.accValue.data[0] * cos(bmi088Cal->Angle.pitch * RAD_PER_DEG) + bmi088Cal->acc.accValue.data[2] * sin(bmi088Cal->Angle.pitch * RAD_PER_DEG);
    bodyxAccFromImu = diffXImuFilter.Apply(bodyxAccFromImu_cal); // 加速度低通滤波
    bodywSpeedFromImu = bmi088Cal->gyro.radps.data[2];
    bodywSpeedFrommMotor = balance.wSpeedFb * OUTER_WHEEL_RADIO / 2;
}
void SlideJudge::init()
{
    diffXspeed.init(&balance.speedFb, 200);
    diffWGyro.init(&bmi088Cal->gyro.radps.data[2], 300);
    diffWspped.init(&balance.wSpeedFb, 200);
    diffXMotorFilter.SetCutoffFreq(2000, 10);
    diffXImuFilter.SetCutoffFreq(2000, 10);
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
    xSpeedErr = balance.chassis->chassisXSpeed - xSpeedEstimate;
    wSpeedErr = bodywSpeedFrommMotor - bodywSpeedFromImu;
    /*通过加速度判断打滑，不能通过加速度判断恢复*/
    if (ABS(xAccErr) > SLIDE_X_ACC_ERR_THREDHOLD &&            // 加速度差阈值
        ABS(bodyxAccFrommMotor) > ABS(bodyxAccFromImu) &&      // 轮子加速度大于身体加速度
        ABS(bodyxAccFrommMotor) > SLIDE_X_ACC_THREDHOLD &&     // 轮子自身加速度阈值
        ABS(bodywSpeedFromImu) < SLIDE_W_SPEED_BODY_THREDHOLD) // 且旋转速度小于阈值
    {
        isXSlide = 1;
    }

    if (ABS(wSpeedErr) > SLIDE_W_SPEED_ERR_THREDHOLD &&       // 加速度差阈值
        ABS(bodywSpeedFrommMotor) > ABS(bodywSpeedFromImu) && // 轮子加速度大于身体加速度
        ABS(bodywSpeedFrommMotor) > SLIDE_W_SPEED_THREDHOLD)  // 轮子自身加速度阈值
    {
        isWSlide = 1;
    }

    /*通过速度判断是否恢复不打滑*/
    if (isSlide) // 已经开始打滑
    {
        if (ABS(xSpeedErr) < SLIDE_X_SPEED_ERR_THREDHOLD ||
						ABS(balance.chassis->chassisXSpeed) < SLIDE_X_SPEED_THREDHOLD) // 估算速度和当前轮速误差较小
        {
            isXSlide = 0;
        }
        if (ABS(wSpeedErr) < SLIDE_W_SPEED_ERR_THREDHOLD)
        {
            isWSlide = 0;
        }
    }
    /*通过两个判定条件确定打滑状态*/
    if (isXSlide || isWSlide)
    {
        slideJudgeTime++;
			if(slideJudgeTime >= SLIDE_JUDGE_TIME)
			{
				isSlide = 1;
				slideJudgeTime = SLIDE_JUDGE_TIME; // 打滑后设置最短的打滑时间
			}
    }
    else
    {
        if (slideJudgeTime > 0)
        {
            slideJudgeTime--;
        }
				else
				{
					 isSlide = 0;
				}
    }

    if (isSlide) // 计数时间大于阈值则判断为打滑，阈值的设置为加速度误判裕度
    {
        balance.chasTorKpBySlide = 0;          // 打滑后减小轮输出（可以做斜坡函数或者速度PID）
    }
    else// 打滑计数减到一定阈值恢复正常
    {
        balance.chasTorKpBySlide = 1; // 打滑恢复后调整轮输出
       
    }
}