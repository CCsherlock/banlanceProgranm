#include "lqrCtrl_task.h"
#include "usart3.h"
#include "iwdg.h"
#include "flash.h"
#include "stdio.h"
#include "imu_task.h"
#include "my_math.h"
#include "dbus.h"
#include "filter.h"
#include "beep.h"
#include "fallRecovery_task.h"
LqrCtrl balance;       // lqr运算实例化
float laqK_buffer[40]; // lqrK矩阵暂存数组
LqrCtrl::LqrCtrl(/* args */)
{
}
/**
 * @brief 初始化lqr所需数据来源
 *  初始LQR算法矩阵 电机 陀螺仪参数
 *  规定两侧电机反馈值同向同号
 *                        ---> + 内圈正方向                       --->+车体方向
 *  外圈正方向 + <--- |||---------------|||   L 左侧            |----------|
 *                     |                 |                      |		  		 |
 *                     |     R       000 |   --->+车体方向      |	  |==|\	 |
 *                     |                 |                      |   |==| \ |  俯﹢仰﹣
 *  外圈正方向 + <--- |||---------------|||   R 右侧						|----------|
 *                        ---> + 内圈正方向
 */
void LqrCtrl::LqrInit()
{
    roboLqr = new Lqr(5, 4);
    chassis = new Chassis;
    roboLqr->lqrInit();
    chassis->chassisInit();
    chassis->setChassisOutPutDir(-1, 1);
    chassis->setlegOutPutDir(1, -1);
    chassis->setChassisFbDir(1 * chassisFbPossitive, -1 * chassisFbPossitive);
    chassis->setLegFbDir(1 * legFbPossitive, -1 * legFbPossitive);
}
/**
 * @brief lqr算法执行
 *        获取反馈->获取设定值->lqr计算->输出
 */

void LqrCtrl::lqrCalRun()
{
    getAllFbValue();
    getAllSetValue();
    roboLqr->calLqrResult();
    if (falling.nowFallSate != COMPLETE_FALL)
    {
        lqrOutput();
    }
}
/**
 * @brief 获取所有反馈值并传入LQR模块
 *
 */
#define JS 0
#if JS
float encodeSpeed_JS;
float motorSpeed_JS;
float encodeAngle_JS;
#endif
void LqrCtrl::getAllFbValue()
{
    getXfb();
    getThetaFb();
    getYawFb();
#if JS
    encodeSpeed_JS = -1 * legMotor[RIGHT]->megSpeed;
    motorSpeed_JS = chssisMotor[LEFT]->motorInfo.motor_fdb.speed_temp;
    encodeAngle_JS = legMotor[RIGHT]->megAngle;
#endif
    fbValue[roboLqr->X] = xFb;
    fbValue[roboLqr->X_DOT] = speedFb;
    fbValue[roboLqr->W] = yawFb;
    fbValue[roboLqr->W_DOT] = yawSpeedFb;
    fbValue[roboLqr->THETA_LEFT] = angleFb[LEFT];
    fbValue[roboLqr->THETA_LEFT_DOT] = angleSpeedFb[LEFT];
    fbValue[roboLqr->THETA_RIGHT] = angleFb[RIGHT];
    fbValue[roboLqr->THETA_RIGHT_DOT] = angleSpeedFb[RIGHT];
    fbValue[roboLqr->FI] = fiFb;
    fbValue[roboLqr->FI_DOT] = fiSpeedFb;
    roboLqr->getFbValue(fbValue, sizeof(fbValue));
}
/**
 * @brief 获取所有设定值并传入LQR模块
 *
 */
void LqrCtrl::getAllSetValue()
{
    setValue[roboLqr->X] = xSet = xFb;
    setValue[roboLqr->X_DOT] = speedSet;
    setValue[roboLqr->W] = yawSet;
    setValue[roboLqr->W_DOT] = yawSpeedSet;
    setValue[roboLqr->THETA_LEFT] = angleSet[LEFT];
    setValue[roboLqr->THETA_LEFT_DOT] = angleSpeedSet[LEFT];
    setValue[roboLqr->THETA_RIGHT] = angleSet[RIGHT];
    setValue[roboLqr->THETA_RIGHT_DOT] = angleSpeedSet[RIGHT];
    setValue[roboLqr->FI] = fiSet;
    setValue[roboLqr->FI_DOT] = fiSpeedSet;
    roboLqr->getSetValue(setValue, sizeof(setValue));
}
void LqrCtrl::getXfb()
{
/* code */
#if defined BIG_MODEL
    xFb = (chassis->getChassisAngel()[LEFT] + chassis->getChassisAngel()[RIGHT]) / 2 * WHEEL_RADIAN / 1000.0f;     // 单位 m
    speedFb = (chassis->getChassisSpeed()[LEFT] + chassis->getChassisSpeed()[RIGHT]) / 2 * WHEEL_RADIAN / 1000.0f; // 单位 m/s
#else
    xFb[i] = chassis->getChassisAngel()[i] * RAD_PER_DEG;          // 单位 rad
    speedFb[i] = rpmToRadps(chassis->getChassisSpeed()[i]) * 0.25; // 单位 m/s
#endif
}
void LqrCtrl::getThetaFb()
{
    getFiFb();
    for (u8 i = 0; i < 2; i++)
    {
/* code */
#if defined BIG_MODEL
        angleFb[i] = (chassis->getLegAngel()[i] - fiFb * legFbPossitive); // 单位 rad
#else
        angleFb[i] = -(chassis->getLegAngel()[i] * RAD_PER_DEG + fiFb); // 单位 rad
#endif
        angleSpeedFb[i] = (chassis->getLegSpeed()[i] - fiSpeedFb * legFbPossitive); // 单位 rad/s
    }
}
void LqrCtrl::getFiFb()
{
    fiFb = (bmi088Cal->Angle.pitch * RAD_PER_DEG * -1) - balance.fiOffset; // 单位 rad
    fiSpeedFb = bmi088Cal->gyro.radps.data[1] * -1;                        // 单位 rad/s
}
/**
 * @brief yaw轴数据反馈 从车上方看，顺时针角度变大，角速度为﹢
 *
 */
void LqrCtrl::getYawFb()
{
    yawFb = -1 * bmi088Cal->Angle.yaw * RAD_PER_DEG;
    yawSpeedFb = -1 * bmi088Cal->gyro.radps.data[2];
}
float chassisTq[2] = {0, 0};
float legTq[2] = {0, 0};
#if defined BIG_MODEL
float legResultKp = 0;     // 调试使用的输出比例
float chassisResultKp = 0; // 调试使用的输出比例
#else

#define FORCE_DEFORCE 0 // 强制脱力调试

#if FORCE_DEFORCE
float legResultKp = 0;
float chassisResultKp = 0;
#else
float legResultKp = 0.5;
float chassisResultKp = 1;
#endif

#endif
#if OUTPUT_TEST
uint8_t resetZeroFlag = 0;
#endif
void LqrCtrl::lqrOutput()
{
#if OUTPUT_TEST

    chassisTq[0] = (RC.Key.CH[1] / 660.0f) * 5;
    chassisTq[1] = (RC.Key.CH[1] / 660.0f) * 5;
    legTq[0] = (RC.Key.CH[3] / 660.0f) * -5;
    legTq[1] = (RC.Key.CH[3] / 660.0f) * 5;

    chassis->chassisCtrlTorque(chassisTq);
    chassis->legCtrlTorque(legTq);
#else
    chassisTorque[LEFT] = LIMIT(roboLqr->resultValue[roboLqr->OUT_LEFT_MOTOR], -MAX_CHASSIS_T, MAX_CHASSIS_T);
    chassisTorque[RIGHT] = LIMIT(roboLqr->resultValue[roboLqr->OUT_RIGHT_MOTOR], -MAX_CHASSIS_T, MAX_CHASSIS_T);
    chassisTorque[LEFT] = chassisTorque[LEFT] * chassisSetPossitive * OUTER_WHEEL_RADIO * chassisResultKp * chasTorKpByMotion;   // 底盘输出力矩乘以系数
    chassisTorque[RIGHT] = chassisTorque[RIGHT] * chassisSetPossitive * OUTER_WHEEL_RADIO * chassisResultKp * chasTorKpByMotion; // 底盘输出力矩乘以系数
    chassis->chassisCtrlTorque(chassisTorque);

    legTorque[LEFT] = LIMIT(roboLqr->resultValue[roboLqr->IN_LEFT_MOTOR], -MAX_LEG_T, MAX_LEG_T);
    legTorque[RIGHT] = LIMIT(roboLqr->resultValue[roboLqr->IN_RIGHT_MOTOR], -MAX_LEG_T, MAX_LEG_T);
    legTorque[LEFT] = legTorque[LEFT] * legSetPossitive * legResultKp * legTorKpByMotion;   // 腿部输出力矩乘以系数
    legTorque[RIGHT] = legTorque[RIGHT] * legSetPossitive * legResultKp * legTorKpByMotion; // 腿部输出力矩乘以系数
    chassis->legCtrlTorque(legTorque);
#endif
}
bool lqrTaskInit = false;
void lqrRunTask()
{
    if (!lqrTaskInit)
    {
        balance.LqrInit(); // 平衡算法初始化 电机初始化
        lqrTaskInit = true;
    }
    saveLqrMessage(); // 不断检测是否有LQR参数变更
    readLqrMessage(); // 如果LQR参数改变，重新读取
    setMegBoardZero();
    balance.lqrCalRun(); // LQR算法计算
}
#define FLASH_SAVE_ADDR ((u32)0x080E0000)
#define FLASH_SAVE_ADDR2 ((u32)0x080D0000)
#define FLASH_DATA_LEN sizeof(laqK_buffer)
bool readflag = 1;
/**
 * @brief 存储LQR参数
 *
 */
void saveLqrMessage()
{
    if (custom_RecvStruct.lqrKChange == 1)
    {
        u32 temp[FLASH_DATA_LEN / 4];
        memcpy(&temp, custom_RecvStruct.lqrK, FLASH_DATA_LEN);
        if (custom_RecvStruct.lqr_plan == 0)
        {
            STMFLASH_Write(FLASH_SAVE_ADDR, temp, FLASH_DATA_LEN / 4);
        }
        else if (custom_RecvStruct.lqr_plan == 1)
        {
            STMFLASH_Write(FLASH_SAVE_ADDR2, temp, FLASH_DATA_LEN / 4);
        }
        custom_RecvStruct.lqrKChange = 0;
        readflag = 1;
        beepDbug(1);
    }
}
/**
 * @brief FLASH 读取LQR参数
 *
 */
void readLqrMessage()
{
    if (readflag)
    {
        u8 datatemp[FLASH_DATA_LEN];
        STMFLASH_Read(FLASH_SAVE_ADDR, (u32 *)datatemp, FLASH_DATA_LEN / 4);
        memcpy(&laqK_buffer, datatemp, FLASH_DATA_LEN);
        memcpy(&balance.roboLqr->lqrK[balance.roboLqr->DOWN_PARAM], laqK_buffer, FLASH_DATA_LEN);
        STMFLASH_Read(FLASH_SAVE_ADDR2, (u32 *)datatemp, FLASH_DATA_LEN / 4);
        memcpy(&laqK_buffer, datatemp, FLASH_DATA_LEN);
        memcpy(&balance.roboLqr->lqrK[balance.roboLqr->UP_PARAM], laqK_buffer, FLASH_DATA_LEN);
        delayMs(10);
        readflag = 0;
        beepDbug(2);
    }
}
int setZeroFlag = 0;
void setMegBoardZero()
{
    if (setZeroFlag)
    {
        legMotor[LEFT]->setMegZeroOffset();
        legMotor[RIGHT]->setMegZeroOffset();
        balance.fiOffset = 0;
        balance.fiOffset = (bmi088Cal->Angle.pitch * RAD_PER_DEG * -1);
        setZeroFlag = 0;
    }
}
