#ifndef _LQR_H_
#define _LQR_H_

#include "board.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
#define CON_VAL_NUM 5 // 反馈变量个数
#define CTR_VAL_NUM 4 // 控制变量个数
class Lqr
{
private:
    /* data */

public:
    enum Status
    {
        X = 0,
        X_DOT,
        W,
        W_DOT,
        THETA_LEFT,
        THETA_LEFT_DOT,
        THETA_RIGHT,
        THETA_RIGHT_DOT,
        FI,
        FI_DOT,
        STATUS_NUM
    };
		enum Param_Index
		{
			DOWN_PARAM = 0,
			UP_PARAM,
			PARAM_NUMBER
		};
    enum Result
    {
        OUT_LEFT_MOTOR = 0,
        OUT_RIGHT_MOTOR,
        IN_LEFT_MOTOR,
        IN_RIGHT_MOTOR,
        RESULT_NUM
    };
    Lqr(int conditionvalueNum, int controlValueNum);
    mat lqr_K[PARAM_NUMBER];
    void lqrInit();
    void getLqrK(float *_LqrValue);
    uint8_t lqrKLoadFlag = 0;
    uint8_t lqrKChangeFlag = 0;
		Param_Index nowLqrParamPlan = DOWN_PARAM;
    mat fd_Value;
    mat set_Value;
    mat err_Value;
    mat result_Value;
    void calLqrResult();
    bool getLqrKMatrix = 0;
    float lqrK[PARAM_NUMBER][CTR_VAL_NUM * (CON_VAL_NUM * 2)];
    float fdValue[CON_VAL_NUM * 2];
    float setValue[CON_VAL_NUM * 2];
    float errValue[CON_VAL_NUM * 2];
    float resultValue[CTR_VAL_NUM];
    void getFbValue(float *_fbvalue, u8 size);
    void getSetValue(float *_setValue, u8 size);
		void setNowParam(Param_Index _setParam);
};

#endif