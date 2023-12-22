#include "beep.h"
Beep beep;
void buzzerPWMInit(u32 arr, u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_Cmd(TIM4, ENABLE);
}

Beep::Beep(/* args */)
{
}
void Beep::beepOn()
{
	beepState = RING;
	TIM_PrescalerConfig(TIM4, _psc, TIM_PSCReloadMode_Immediate);
	TIM_SetCompare3(TIM4, _pwm);
}
void Beep::beepOff()
{
	beepState = SILENT;
	TIM_PrescalerConfig(TIM4, MAX_PSC, TIM_PSCReloadMode_Immediate);
	TIM_SetCompare3(TIM4, _pwm);
}
void Beep::setBeepTone(uint16_t tone)
{
	_psc = tone;
}
void Beep::beepToggle()
{
	if (beepState == RING)
	{
		beepOff();
	}
	else
	{
		beepOn();
	}
}
void Beep::beepAlarm(uint8_t times)
{
	if (ringLoopReset)
	{
		runingTime = (float)(getSysTimeUs() / 1e3f);
		runingTime_last = runingTime;
		ringLoopTimes = 0;
		ringLoopReset = false;
		beepOff();
		return;
	}
	runingTime = (float)(getSysTimeUs() / 1e3f);
	if (ringLoopTimes >= times * 2)
	{
		if(runingTime - runingTime_last<ringTime*4)
		{
			beepOff();
			return;
		}
		ringLoopReset = true;
		return;
	}
	if (runingTime - runingTime_last > ringTime)
	{
		beepToggle();
		runingTime_last = runingTime;
		ringLoopTimes++;
	}
}