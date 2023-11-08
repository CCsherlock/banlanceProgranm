#ifndef _UT_MOTOR_
#define _UT_MOTOR_

#include "board.h"
#include "crc.h"
#include "rs485.h"

#define MaxToleranceLostNum 5 //最大允许的丢包数
#define REDUCTION_RATIO 9.1f //电机减速比

/* 规定： CalAgl = Dir * (OrgAgl / REDUCTION_RATIO + Offset) */
/*  	  OrgAgl =( CalAgl/Dir - Offset ) * REDUCTION_RATIO  */
/*  	  Offset = CalAgl/Dir - OrgAgl / REDUCTION_RATIO     */

/********************************************************************/

//电机发送数数据
struct MOTOR_send_t{

    //待发送的各项数据
    unsigned short mode; //0:空闲, 5:开环转动, 6:奇怪模式, 10:闭环FOC控制

    //实际给FOC的指令力矩为：K_P*delta_Pos + K_W*delta_W + T
    float T; 	//期望关节的输出力矩（电机本身的力矩）（Nm）
    float W; 	//期望关节速度（电机本身的速度）(rad/s)
    float Pos; 	//期望关节位置（rad）
    float K_P; 	//关节刚度系数
    float K_W; 	//关节速度系数

    uint32_t Res;
};

//电机接受数据
struct MOTOR_recv_t{

    //解读得出的电机数据
    unsigned char motor_id; //电机ID
    unsigned char mode; //0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp; //温度
    unsigned char MError; //错误码

    float T; 	// 当前实际电机输出力矩
    float W; 	// 当前实际电机速度（高速）
    float LW; 	// 当前实际电机速度（低速）
    int Acc; 	// 电机转子加速度
    float Pos; 	// 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

    float gyro[3]; // 电机驱动板6轴传感器数据
    float acc[3];
};

/********************************************************************/

//宇树电机类
class UT_Motor{
    public:
        UT_Motor(uint8_t id, uint8_t ch, int8_t dir = 1);

        bool motorTempErr; //电机温度异常位
        bool motorLostFlag = 1; //电机丢失标志位
        float motorPosOffset; //电机位置偏置
	
		/**
		 * @brief 获取电机设置的方向
		 */
        inline int8_t dir()
		{
			return motorInfo.dir;
		};
		float caledAgl(void);
        void motorCtrl(float T,float W,float Pos,float K_P,float K_W);
        void motorRxData_Handle(Recv_Struct_t* rs);

        MOTOR_recv_t MOTOR_recv;

    private:
        MOTOR_send_t MOTOR_send;

        struct motorInfo_t{
            uint8_t motorID;
            uint8_t ch;
            int8_t dir;
        }motorInfo;
		
        bool motorReadyFlag; //电机可以控制标志位
        uint8_t msgLostTimes; //丢包数

        void deforce();
        void onforce();
        void motorTxData_Handle();
};

/********************************************************************/

#endif
