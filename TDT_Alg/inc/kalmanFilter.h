#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

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

typedef struct kf_t
{
    float *FilteredValue;  // 估计值
    float *MeasuredVector; // 测量值
    float *ControlVector;  // 控制输入

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;      // 测量值与状态的对应关系
    float *MeasurementDegree;     // 测量值对应H矩阵元素值
    float *MatR_DiagonalElements; // 量测方差
    float *StateMinVariance;      // 最小方差 避免方差过度收敛
    uint8_t *temp;
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    mat xhat;      // x(k|k) 后验估计 x(k|k)
    mat xhatminus; // x(k|k-1) 先验估计 x(k|k-1)
    mat u;         // control vector u 控制向量 u
    mat z;         // measurement vector z 量测向量 z
    mat P;         // covariance matrix P(k|k) 后验协方差矩阵 P(k|k)
    mat Pminus;    // covariance matrix P(k|k-1) 先验协方差矩阵 P(k|k-1)
    mat F, FT;     // state transition matrix F FT 状态转移矩阵 F FT
    mat B;         // control matrix B 控制矩阵 B
    mat H, HT;     // measurement matrix H 量测矩阵 H
    mat Q;         // process noise covariance matrix Q 过程噪声矩阵 Q
    mat R;         // measurement noise covariance matrix R 量测噪声矩阵 R
    mat K;         // kalman gain  K kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;
    // 用户函数指针
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);
    // 矩阵数据
    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;
extern uint16_t sizeof_float, sizeof_double;

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif //__KALMAN_FILTER_H
