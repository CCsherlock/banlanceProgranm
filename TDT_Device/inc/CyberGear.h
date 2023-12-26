#ifndef _CYBER_GEAR_H_
#define _CYBER_GEAR_H_
#include "board.h"
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f
#define MAX_P 4 * PI
#define MIN_P -4 * PI

#define Master_CAN_ID 0x00

#define Communication_Type_GetID 0x00         // 获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01 // 用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02  // 用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03   // 电机使能运行
#define Communication_Type_MotorStop 0x04     // 电机停止运行
#define Communication_Type_SetPosZero 0x06    // 设置电机机械零位
#define Communication_Type_CanID 0x07         // 更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11 // 读取单个参数
#define Communication_Type_SetSingleParameter 0x12 // 设定单个参数
#define Communication_Type_ErrorFeedback 0x15      // 故障反馈帧

#define Run_mode 0x7005
#define Iq_Ref 0x7006
#define Spd_Ref 0x700A
#define Limit_Torque 0x700B
#define Cur_Kp 0x7010
#define Cur_Ki 0x7011
#define Cur_Filt_Gain 0x7014
#define Loc_Ref 0x7016
#define Limit_Spd 0x7017
#define Limit_Cur 0x7018

#define Gain_Angle 720 / 32767.0
#define Bias_Angle 0x8000
#define Gain_Speed 30 / 32767.0
#define Bias_Speed 0x8000
#define Gain_Torque 12 / 32767.0
#define Bias_Torque 0x8000
#define Temp_Gain 0.1

#define Motor_Error 0x00
#define Motor_OK 0X01

#define Current_mode 3
#define Position_mode 1
#define Motion_mode 0
#define Speed_mode 2

#define MEG_BOARD_CANID 0x100
#define MEG_BOARD_CANX CAN2
struct MI_Motor
{
    uint8_t CAN_ID;
    CAN_TypeDef *can_x;
    int Motor_Nbr;

    float Angle;
    float Speed;
    float Torque;
    float Temp;

    uint16_t set_current;
    uint16_t set_speed;
    uint16_t set_position;

    uint8_t error_code;

    float Angle_Bias;
};
enum motor_mode_e // 电机运行模式
{
    RESET_MODE = 0, // Reset[模式]
    CALI_MODE = 1,  // Cali 模式[标定]
    RUN_MODE = 2    // Motor模式[运行]
};
struct EXT_ID_t // 32位扩展ID解析结构体
{
    uint32_t motor_id : 8; // 只占8位
    uint32_t data : 16;
    uint32_t mode : 5;
    uint32_t res : 3;
};
struct Motor_fdb_t // 电机编码器反馈结构体
{
    // 电机反馈
    float angle_temp;
    float speed_temp;
    float torque_temp;
    float temprature_temp;

    float angle; // 连续角
    float speed;
    float torque;
    float temprature;
    uint32_t last_update_time; // 编码器时间戳
};
enum motor_state_e // 电机状态（故障信息）
{
    OK = 0,                // 无故障
    BAT_LOW_ERR = 1,       // 欠压故障
    OVER_CURRENT_ERR = 2,  // 过流
    OVER_TEMP_ERR = 3,     // 过温
    MAGNETIC_ERR = 4,      // 磁编码故障
    HALL_ERR_ERR = 5,      // HALL编码故障
    NO_CALIBRATION_ERR = 6 // 未标定
};

struct MI_Motor_t
{
    CAN_TypeDef *phcan;
    motor_state_e motor_state;
    motor_mode_e motor_mode;
    EXT_ID_t EXT_ID;
    uint8_t txdata[8];
    uint8_t lostFlag;
    uint16_t lostCnt;
    Motor_fdb_t motor_fdb;
};
struct MegEncode_t
{
    uint8_t resetFlag = 0;
    uint8_t setZeroFlag = 0;
    uint8_t lostFlag;
    uint16_t lostCnt;
};

class CyberGear
{
private:
    /* data */
    CAN_TypeDef *myCan_x;
    uint32_t _extID;
    uint32_t _megBoardID;
    int _motorNum;
    float _runMode;

public:
    MI_Motor_t motorInfo;
    CyberGear(CAN_TypeDef *_Canx, uint8_t _Ext_ID, uint8_t _Meg_ID, int Motor_Num, float mode);
    void initMotor();
    void enableMotor();
    void stopMotor(uint8_t clear_error);
    void motorDataHandler(CanRxMsg *canRxData);
    uint32_t getMotorID(uint32_t CAN_ID_Frame);
    void setMotorParameter(uint16_t index, uint8_t data[4]);
    void motorCtrlMode(float torque, float MechPosition, float speed, float kp, float kd);
    void setZeroPos();
    void setCANID(uint8_t Target_ID);
    void megSpeedMessegeGet(CanRxMsg *canRxData);
    float megSpeed;
    float megAngle;
    void setMegZeroOffset();
    void resetMegBoard();
    MegEncode_t megTrans;
};
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
uint8_t *Float_to_Byte(float f);
void cyberGearLostCheck(CyberGear* motor);
typedef struct _vec4u
{
    uint8_t data[4];
} _vec4u;
typedef union
{
    uint8_t U[4];
    float F;
    int I;
} FormatTrans;
#endif