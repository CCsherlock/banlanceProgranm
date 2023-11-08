#include "lqr.h"
#include "usart3.h"
#include "flash_var.h"
#include "iwdg.h"
Lqr::Lqr(int conditionvalueNum, int controlValueNum)
{
}
/**
 * @brief Lqr初始化矩阵K Fb Set Result
 *
 */
void Lqr::lqrInit()
{
    if (!getLqrKMatrix)
    {
        Matrix_Init(&lqr_K, CTR_VAL_NUM, CON_VAL_NUM * 2, &lqrK[0]);
        Matrix_Init(&fd_Value, CON_VAL_NUM * 2, 1, &fdValue[0]);
        Matrix_Init(&set_Value, CON_VAL_NUM * 2, 1, &setValue[0]);
        Matrix_Init(&err_Value, CON_VAL_NUM * 2, 1, &errValue[0]);
        Matrix_Init(&result_Value, CTR_VAL_NUM, 1, &resultValue[0]);
        getLqrKMatrix = 1;
    }
}
/**
 * @brief 获取Lqr K矩阵
 *
 * @param _LqrValue 传入K矩阵的首地址
 */
void Lqr::getLqrK(float *_LqrValue)
{
    lqrKChangeFlag = custom_RecvStruct.lqrKChange;
    if (!lqrKChangeFlag)
    {
        return;
    }
    if (lqrKLoadFlag)
    {
        return;
    }
    for (u8 i = 0; i < (CTR_VAL_NUM * (CON_VAL_NUM * 2)); i++)
    {
        lqrK[i] = *(custom_RecvStruct.lqrK + i);
        lqrKLoadFlag = 1;
    }
//				// 看门狗复位时间1.5s
//				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
//				auto oldIWDG_P = IWDG->PR;
//				auto oldIWDG_RL = IWDG->RLR;
//				IWDG_SetPrescaler(IWDG_Prescaler_64); // 设置IWDG分频系数
//				IWDG_SetReload(750);				  // 设置IWDG装载值
//				IFlash.save();
//				// 恢复看门狗时间
//				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对IWDG->PR IWDG->RLR的写
//				IWDG_SetPrescaler(oldIWDG_P);				  // 设置IWDG分频系数
//				IWDG_SetReload(oldIWDG_RL);					  // 设置IWDG装载值
//				iwdgFeed();									  // reload
//				__set_FAULTMASK(1);							  // 关闭所有中断
//				NVIC_SystemReset();							  // 复位
//				while (1)
//				{
//				} // 仅等待复位
}
/**
 * @brief 获取反馈值
 *
 * @param _fbvalue 反馈值数组的首地址
 */
void Lqr::getFbValue(float *_fbvalue, u8 size)
{
    if (sizeof(fdValue) != size)
    {
        return;
    }
    for (u8 i = 0; i < CON_VAL_NUM * 2; i++)
    {
        fdValue[i] = *(_fbvalue + (i));
    }
}
/**
 * @brief 获取设定值
 *
 * @param _setValue 设定值数组的首地址
 */
void Lqr::getSetValue(float *_setValue, u8 size)
{
    if (sizeof(setValue) != size)
    {
        return;
    }
    for (u8 i = 0; i < CON_VAL_NUM * 2; i++)
    {
        setValue[i] = *(_setValue + (i));
    }
}
/**
 * @brief lqr算法计算
 *        result  = k * (set - fb)
 *
 */
void Lqr::calLqrResult()
{
    // TODO  判断设定值和反馈值nan和inf
    Matrix_Subtract(&set_Value, &fd_Value, &err_Value);
//    Matrix_Multiply(&err_Value, &lqr_K, &result_Value);
    // TODO  判断输出值nan和inf
}