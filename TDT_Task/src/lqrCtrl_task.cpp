#include "lqrCtrl_task.h"
#include "usart3.h"
#include "flash_var.h"
#include "iwdg.h"
LqrCtrl balance;
float laqK_buffer[40];
LqrCtrl::LqrCtrl(/* args */)
{
}
void LqrCtrl::LqrInit()
{
    roboLqr = new Lqr(5, 4);
    chassis = new Chassis;
}
void LqrCtrl::lqrCalRun()
{
    if (!lqrInitflag)
    {
        lqrInitflag = true;
    }
    // roboLqr->getLqrK(custom_RecvStruct.lqrK);
    getAllFbValue();
    getAllSetValue();
    roboLqr->calLqrResult();
    lqrOutput();
}
void LqrCtrl::getAllFbValue()
{
    getSpeedFb();
    getThetaFb();
    fbValue[roboLqr->X_LEFT] = xFb[LEFT];
    fbValue[roboLqr->X_LEFT_DOT] = speedFb[LEFT];
    fbValue[roboLqr->X_RIGHT] = xFb[RIGHT];
    fbValue[roboLqr->X_RIGHT_DOT] = speedFb[RIGHT];
    fbValue[roboLqr->THETA_LEFT] = angleFb[LEFT];
    fbValue[roboLqr->THETA_LEFT_DOT] = angleSpeedFb[LEFT];
    fbValue[roboLqr->THETA_RIGHT] = angleFb[RIGHT];
    fbValue[roboLqr->THETA_RIGHT_DOT] = angleSpeedFb[RIGHT];
    fbValue[roboLqr->FI] = fiFb;
    fbValue[roboLqr->FI_DOT] = fiSpeedFb;
    roboLqr->getFbValue(fbValue, sizeof(fbValue));
}
void LqrCtrl::getAllSetValue()
{
    setValue[roboLqr->X_LEFT] = xSet[LEFT] = xFb[LEFT];
    setValue[roboLqr->X_LEFT_DOT] = speedSet[LEFT];
    setValue[roboLqr->X_RIGHT] = xSet[RIGHT] = xFb[RIGHT];
    setValue[roboLqr->X_RIGHT_DOT] = speedSet[RIGHT];
    setValue[roboLqr->THETA_LEFT] = angleSet[LEFT];
    setValue[roboLqr->THETA_LEFT_DOT] = angleSpeedSet[LEFT];
    setValue[roboLqr->THETA_RIGHT] = angleSet[RIGHT];
    setValue[roboLqr->THETA_RIGHT_DOT] = angleSpeedSet[RIGHT];
    setValue[roboLqr->FI] = fiSet;
    setValue[roboLqr->FI_DOT] = fiSpeedSet;
    roboLqr->getSetValue(setValue, sizeof(setValue));
}
void LqrCtrl::getSpeedFb()
{

    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        speedFb[i] = chassis->chssisMotor[i]->canInfo.speed;
    }
}
void LqrCtrl::getThetaFb()
{
    static uint64_t speedTime, speedTimeLast, speedTimeErr;
    speedTime = getSysTimeUs();
    speedTimeErr = timeIntervalFrom(speedTimeLast);
    speedTimeLast = speedTime;
    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        angleFb[i] = chassis->legMotor[i]->MOTOR_recv.Pos + fiFb;
        angleSpeedFb[i] = (angleFb[i] - angleFb_last[i]) / (float)(speedTimeErr / 1e6f);
        angleFb_last[i] = angleFb[i];
    }
}
void LqrCtrl::getFiFb()
{
    static uint64_t fiTime, fiTimeLast, fiTimeErr;
    fiTime = getSysTimeUs();
    fiTimeErr = timeIntervalFrom(fiTimeLast);
    fiTimeLast = fiTime;
    fiFb = 0; // TODO feedbackSource
    fiSpeedFb = (fiFb - fiFb_last) / (float)(fiTimeErr / 1e6f);
    fiFb_last = fiFb;
}
void LqrCtrl::lqrOutput()
{
    float chassisTorque[2];
    chassisTorque[LEFT] = roboLqr->resultValue[roboLqr->OUT_LEFT_MOTOR];
    chassisTorque[RIGHT] = roboLqr->resultValue[roboLqr->OUT_RIGHT_MOTOR];
    chassis->chassisCtrlTorque(chassisTorque);
    float legTorque[2];
    legTorque[LEFT] = roboLqr->resultValue[roboLqr->IN_LEFT_MOTOR];
    legTorque[RIGHT] = roboLqr->resultValue[roboLqr->IN_RIGHT_MOTOR];
    chassis->legCtrlTorque(legTorque);
}
void LqrCtrl::lqrKset()
{
}
void linkLqrFlash()
{
}
bool lqrTaskInit = false;
void lqrRunTask()
{
    if (!lqrTaskInit)
    {
        balance.LqrInit();
       IFlash.link(laqK_buffer, 7);
//			IFlash.read();
        lqrTaskInit = true;
    }
		saveLqrMessage();
    balance.lqrCalRun();
}
bool saveflag = 0;
void saveLqrMessage()
{
    if (saveflag == 1)
    {
//        memcpy(&laqK_buffer, custom_RecvStruct.lqrK, sizeof(laqK_buffer));
//        // 看门狗复位时间1.5s
//        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
//        auto oldIWDG_P = IWDG->PR;
//        auto oldIWDG_RL = IWDG->RLR;
//        IWDG_SetPrescaler(IWDG_Prescaler_64); // 设置IWDG分频系数
//        IWDG_SetReload(750);                  // 设置IWDG装载值
           IFlash.save();
//        // 恢复看门狗时间
//        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
//        IWDG_SetPrescaler(oldIWDG_P);                 // 设置IWDG分频系数
//        IWDG_SetReload(oldIWDG_RL);                   // 设置IWDG装载值
//        iwdgFeed();                                   // reload
//        __set_FAULTMASK(1);                           // 关闭所有中断
//        NVIC_SystemReset();                           // 复位
//        while (1)
//        {
//        } // 仅等待复位
			saveflag = 0;
    }
}