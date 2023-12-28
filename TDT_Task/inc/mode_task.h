#ifndef _MODE_TASK_H_
#define _MODE_TASK_H_

#include "board.h"
#include "angleTranseform.h"
#define ROBOT_MAX_V 1
class RampCurve
{
private:
    enum CurveMode
    {
        INIT = 0,
        RUNNING,
        END
    };

public:
    float curveResult;
    float curveTime, curveTime_last;
    bool curveInit = false;
    RampCurve()
    {
        curveResult = 0;
        curveTime = curveTime_last = 0;
        curveInit = false;
    }
    bool curveFinish;
    float ramp(double slop, double start, double end)
    {
        if (!curveInit)
        {
            curveResult = start;
            curveTime = (float)(getSysTimeUs() / 1e6f);
            curveTime_last = curveTime;
            curveInit = true;
            curveFinish = false;
            return curveResult;
        }
        curveTime = (float)(getSysTimeUs() / 1e6f);
        if (signbit(slop))
        {
            if (curveResult <= end)
            {
                curveMode = END;
                curveFinish = true;
                return end;
            }
        }
        else
        {
            if (curveResult >= end)
            {
                curveMode = END;
                curveFinish = true;
                return end;
            }
        }

        double timeErr = curveTime - curveTime_last;
        curveResult = curveResult + slop * timeErr;
        curveTime_last = curveTime;
        curveMode = RUNNING;
        return curveResult;
    }
    void reset()
    {
        if (curveMode != INIT)
        {
            curveResult = 0;
            curveTime = curveTime_last = 0;
            curveMode = INIT;
            curveInit = false;
        }
    }
    uint8_t curveMode;
};

enum RobotMotion
{
    DEFORCE = 0U, // 脱力
    SIT,          // 坐姿
    CROSS_STAND,  // 交叉站姿
    ALL_MODE_NUM
};
class RunMode
{
private:
    /* data */

public:
    RunMode(RobotMotion _thisMode);
    virtual uint8_t intoModeRun(RobotMotion _modeLast) { return 0; };
    virtual void inModeRun(){};
    virtual void reset(){};
    uint8_t transeOverFlag = 0;
    uint8_t transeResetFlag = 0;
    struct RobotCtrlVal
    {
        /* data */
        float chassisSpeed[2];  // 底盘直行速度设定
        float chassisTurnSpeed; // 底盘转向速度设定
        float chassisYaw;       // 底盘Yaw方向设定
        float bodyPitch;        // 机体俯仰设定
        float bodyTheta[2];     // 关节角度设定
    } robotCtrl;
    RobotMotion runMode_last = DEFORCE;
    RobotMotion thisMode;
    // 任务列表
    static RunMode **modeList;
    // 任务数量
    static int modeNum;
};

class DeforceMode : public RunMode
{
private:
    /* data */
public:
    DeforceMode() : RunMode(DEFORCE){};
    uint8_t intoModeRun(RobotMotion _modeLast) override;
    void inModeRun() override;
    void reset() override { transeResetFlag = true; };
};
extern DeforceMode deforceMode;

class SitMode : public RunMode
{
private:
    /* data */
public:
    SitMode(/* args */) : RunMode(SIT){};
    uint8_t intoModeRun(RobotMotion _modeLast) override;
    void inModeRun() override;
    void reset() override;
    RampCurve pitchRamp;
    RampCurve thetaRamp[2];
    uint8_t recodeTranseFlag = 0;
    float thetaStart[2], thetaEnd[2];
};
extern SitMode sitMode;

class CrossStandMode : public RunMode
{
public:
    CrossStandMode() : RunMode(CROSS_STAND){};
    uint8_t intoModeRun(RobotMotion _modeLast) override;
    void inModeRun() override;
    void reset() override;
    RampCurve thetaRamp[2];
    uint8_t recodeTranseFlag = 0;
    float thetaStart[2], thetaEnd[2];
};
extern CrossStandMode standMode;
#endif