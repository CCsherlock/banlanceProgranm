#ifndef _LQRCTRL_TASK_H_
#define _LQRCTRL_TASK_H_

#include "board.h"
#include "lqr.h"
#include "chassis_task.h"

#if defined SMALL_MODLE
#define MAX_CHASSIS_T 1.3
#define MAX_LEG_T 1.2
#else
#define MAX_CHASSIS_T 2
#define MAX_LEG_T 12
#define WHEEL_RADIAN 210/1000.0f
#define CHAASIS_WHEEL_DISS 105/1000.0f
#endif
class LqrCtrl
{
private:
  /* data */
public:
  LqrCtrl(/* args */);
  Lqr *roboLqr;
  Chassis *chassis;
  void LqrInit();
  bool lqrInitflag = false;
  float setValue[10]; // 设定值LQR传参数组
  float fbValue[10];  // 反馈值LQR传参数组
  void getAllFbValue();
  void getAllSetValue();
  void lqrCalRun();
  void lqrOutput();
  void getThetaFb();
  void getFiFb();
  void getXfb();
  void getYawFb();
  bool lqrKsetFlag = false;
  float xSet; // 底盘位置设定值
  float xFb;  // 底盘位置反馈值
  float yawSet;
  float yawFb;
  float speedSet; // 底盘速度设定值
  float speedFb;  // 底盘速度反馈值
  float wSpeedFb; // 由轮速计算的旋转速度
  float yawSpeedSet;
  float yawSpeedFb;
  float angleSet[2] = {0, 0};      // 关节角度设定值
  float angleFb[2] = {0, 0};       // 关节角度反馈值
  float angleSpeedSet[2] = {0, 0}; // 关节角速度设定值
  float angleSpeedFb[2] = {0, 0};  // 关节角速度反馈值
  float fiSet;                     // 机体角度设定值
  float fiSpeedSet;                // 机体角速度设定值
  float fiFb;                      // 机体角度反馈值
  float fiSpeedFb;                 // 机体角速度反馈值
  float chassisTorque[2] = {0, 0};
  float legTorque[2] = {0, 0};
  float legTorKpByMotion;  // 由模式选择器控制的输出比例
  float chasTorKpByMotion; // 由模式选择器控制的输出比例
  //		int chassisSetPossitive =-1;
  //		int legSetPossitive =-1;
#if defined SMALL_MODLE
  int chassisSetPossitive = -1;
  int legSetPossitive = -1;
  int chassisFbPossitive = 1;
  int legFbPossitive = 1;
#else
  int chassisSetPossitive = 1;
  int legSetPossitive = 1;
  int chassisFbPossitive = 1;
  int legFbPossitive = -1;
	float fiOffset = -0.0038092141;
#endif
};
extern float laqK_buffer[40];
void linkLqrFlash();
void saveLqrMessage();
void lqrRunTask();
void readLqrMessage();
void setMegBoardZero();
extern LqrCtrl balance;
#endif