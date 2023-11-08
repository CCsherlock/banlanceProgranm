/*****************************************************************************
File name: TDT_Task\src\led_task.h
Description: 呼吸灯控制任务
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __LED_TASK_H__
#define __LED_TASK_H__

#include "board.h"
#include "led.h"
extern Led boardLed;
#if defined USE_MAIN_CTRL_2019	
extern Led boardLed;
#elif defined USE_MAIN_CTRL_2021_PJ||defined USE_MAIN_CTRL_2021_B	\
	||defined USE_MAIN_CTRL_2021_A||defined USE_MAIN_CTRL_2020
extern Led boardLed;
#elif defined USE_MAIN_CTRL_RM_CBOARD
extern Led boardLed;
#endif
extern Led laser;

void ledInit();
void Led_Task();
#endif
