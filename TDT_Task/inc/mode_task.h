#ifndef _MODE_TASK_H_
#define _MODE_TASK_H_

#include "board.h"
#include "angleTranseform.h"
#include "lqrCtrl_task.h"
#include "dbus.h"
#include "imu_task.h"
#define ROBOT_MAX_V 30
#define ROBOT_MAX_W 0.01

//#define START_FROM_UP
#define START_FROM_DOWN
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
		JUMP,
    ALL_MODE_NUM
};
class RunMode
{
public:
    RunMode(RobotMotion _thisMode);
    virtual uint8_t intoModeRun(RobotMotion _modeLast) { return 0; };
    virtual void inModeRun(){};
    virtual void reset(){};
    virtual void modeInit(){};
    uint8_t modeInitFlag = false;
    uint8_t transeOverFlag = 0;
    uint8_t transeResetFlag = 0;
    struct RobotCtrlVal
    {
        /* data */
        double chassisSpeed;  // 底盘直行速度设定
        double chassisYaw;       // 底盘Yaw方向设定
        double bodyPitch;        // 机体俯仰设定
        double bodyTheta[2];     // 关节角度设定
        float chaTorqueKp = 1;
        float legTorqueKp = 1;
    } robotCtrl;
    RobotMotion runMode_last = DEFORCE;
    RobotMotion thisMode;
    // 任务列表
    static RunMode **modeList;
    // 任务数量
    static int modeNum;
};


#endif