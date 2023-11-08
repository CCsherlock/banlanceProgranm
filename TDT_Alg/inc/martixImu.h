#ifndef _MARTIX_IMU_H_
#define _MARTIX_IMU_H_

#include "board.h"

class KalmanImu
{
private:
    /* data */
    bool initFlag = 0;
public:
    KalmanImu(/* args */);
    void gEstimateKF_Init(float process_noise, float measure_noise);
    void gEstimateKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void kalmanImuCal();
};



extern KalmanImu myKalman;


#endif