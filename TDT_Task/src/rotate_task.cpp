#include "rotate_task.h"
#include "dbus.h"
float motorPidParaInner[2] = {2, 0.5};
float motorPidParaOuter[2] = {3, 0};
#define STATIC 0
#define ROTATE 1
#define ROTATE_SLOW 2
#define RATIO 19
RotateTask rotate;
RotateTask::RotateTask(/* args */)
{
    motor = new Motor(M3508, CAN1, 0x201);
    motorPidInner = new PidParam;
    positionPid = new PidParam;
}

void RotateTask::taskInit()
{

    motorPidInner->kp = motorPidParaInner[0];
    motorPidInner->ki = motorPidParaInner[1];
    motorPidInner->integralErrorMax = motor->getMotorCurrentLimit();
    motorPidInner->resultMax = motor->getMotorCurrentLimit();

    motor->pidInner.paramPtr = motorPidInner;
    motor->pidInner.fbValuePtr[0] = &motor->canInfo.speed;

    positionPid->kp = motorPidParaOuter[0];
    positionPid->ki = motorPidParaOuter[1];
    positionPid->integralErrorMax = motor->getMotorSpeedLimit();
    positionPid->resultMax = motor->getMotorSpeedLimit();

    motor->pidOuter.paramPtr = positionPid;
    motor->pidOuter.fbValuePtr[0] = &angleTotal;

    if (motor->canInfo.lostFlag == 1)
    {
        return;
    }
    else
    {
        initPosition = motor->canInfo.totalEncoder;
        motor->setZeroValue(initPosition);
        initFlag = true;
    }
}
void RotateTask::run()
{
    if (initFlag != true)
    {
        taskInit();
        return;
    }
    else
    {
        if (deforceFlag)
        {
            return;
        }
        stateSwitch();
        rotateSpeedCal();
        positionCal();
        if (stateFlag == STATIC)
        {

            motor->ctrlPosition(positionSet);
        }
        else if (stateFlag == ROTATE)
        {
            motor->ctrlSpeed(speedSet);
        }
        else if (stateFlag == ROTATE_SLOW)
        {
            motor->ctrlSpeed(speedSet * 0.5);
        }
    }
}
void RotateTask::rotateSpeedCal()
{
    // speedSet = (0.4 * 60) * 19 * RATIO;
    speedSet = 1000;
}
void RotateTask::positionCal()
{
    position = motor->canInfo.totalEncoder;
    angleTotal = position / (8192.0f * RATIO) * 360;
    angle = (int)angleTotal % 360;
    if (ABS(angleTotal - positionSet) > 5)
    {
        arriveFlag = 0;
    }
    else
    {
        arriveFlag = 1;
    }
    if (arriveFlag)
    {
        return;
    }
    if (angle > 0)
    {
        if (angle < 180)
        {
            positionSet = angleTotal - angle;
        }
        else
        {
            positionSet = angleTotal - angle + 360;
        }
    }
    else
    {
        if (angle > -180)
        {
            positionSet = angleTotal - angle;
        }
        else
        {
            positionSet = angleTotal - angle - 360;
        }
    }
}
void RotateTask::stateSwitch()
{
    if (RC.Key.CH[4] == 1)
    {
        stateFlag = ROTATE;
    }
    else if (RC.Key.CH[4] == 3)
    {
        stateFlag = ROTATE_SLOW;
    }
    else
    {
        stateFlag = STATIC;
    }
}