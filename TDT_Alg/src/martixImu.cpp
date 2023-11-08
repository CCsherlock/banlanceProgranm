#include "martixImu.h"
#include "kalmanFilter.h"
#include "imu_task.h"
KalmanImu myKalman;
KalmanFilter_t gEstimateKF;	   // 卡尔曼滤波器结构体
float gVec[3];    // 重力加速度向量 用于存储估计值供其他函数调用

float gEstimateKF_F[9] = {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1};	   // 状态转移矩阵其余项在滤波器更新时更新
float gEstimateKF_P[9] = {100, 0.1, 0.1,
                          0.1, 100, 0.1,
                          0.1, 0.1, 100};    // 后验估计协方差初始值
static float gEstimateKF_Q[9] = {0.01, 0, 0,
                                 0, 0.01, 0,
                                 0, 0, 0.01};    // Q矩阵初始值（其实这里设置多少都无所谓）
static float gEstimateKF_R[9] = {100000, 0, 0,
                                 0, 100000, 0,
                                 0, 0, 100000};    // R矩阵初始值（其实这里设置多少都无所谓）
float gEstimateKF_K[9];
const float gEstimateKF_H[9] = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};	// 由于不需要异步量测自适应，这里直接设置矩阵H为常量

void KalmanImu::gEstimateKF_Init(float process_noise, float measure_noise)
{
    for (uint8_t i = 0; i < 9; i += 4)
    {
        // 初始化过程噪声与量测噪声
        gEstimateKF_Q[i] = process_noise;
        gEstimateKF_R[i] = measure_noise;
    }

    Kalman_Filter_Init(&gEstimateKF, 3, 0, 3);	// 状态向量3维 无控制部分 测量向量3维
    // gEstimateKF.User_Func0_f = gEstimateKF_Tuning;
    memcpy(gEstimateKF.F_data, gEstimateKF_F, sizeof(gEstimateKF_F));
    memcpy(gEstimateKF.P_data, gEstimateKF_P, sizeof(gEstimateKF_P));
    memcpy(gEstimateKF.Q_data, gEstimateKF_Q, sizeof(gEstimateKF_Q));
    memcpy(gEstimateKF.R_data, gEstimateKF_R, sizeof(gEstimateKF_R));
    memcpy(gEstimateKF.H_data, gEstimateKF_H, sizeof(gEstimateKF_H));
}

void KalmanImu::gEstimateKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    // 空间换时间 避免重复运算
    static float gxdt, gydt, gzdt;
    gxdt = gx * dt;
    gydt = gy * dt;
    gzdt = gz * dt;

    // 由于本例中状态转移矩阵为时变矩阵
    // 需要在卡尔曼滤波器更新前更新转移矩阵F的值
    gEstimateKF.F_data[1] = gzdt;
    gEstimateKF.F_data[2] = -gydt;

    gEstimateKF.F_data[3] = -gzdt;
    gEstimateKF.F_data[5] = gxdt;

    gEstimateKF.F_data[6] = gydt;
    gEstimateKF.F_data[7] = -gxdt;

    // 卡尔曼滤波器测量值更新
    // 不一定写在滤波器更新函数之前，也可写在与传感器通信的回调函数中
    gEstimateKF.MeasuredVector[0] = ax;
    gEstimateKF.MeasuredVector[1] = ay;
    gEstimateKF.MeasuredVector[2] = az;

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(&gEstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 3; i++)
    {
        gVec[i] = gEstimateKF.FilteredValue[i];
    }
}
void KalmanImu::kalmanImuCal()
{
    if(!initFlag)
    {
        gEstimateKF_Init(0.05,0.1);
        initFlag = 1;
    }
//    gEstimateKF_Update(mpu6050Cal->gyro.calibration.data[0],mpu6050Cal->gyro.calibration.data[1],mpu6050Cal->gyro.calibration.data[2],
//                        mpu6050Cal->acc.accValue.data[0], mpu6050Cal->acc.accValue.data[1], mpu6050Cal->acc.accValue.data[2],0.002);
}
KalmanImu::KalmanImu(/* args */)
{
}