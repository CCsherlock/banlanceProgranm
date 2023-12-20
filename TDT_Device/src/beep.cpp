#include "beep.h"
//定时器4 通道3 PD14
void buzzerPWMInit(u32 arr,u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTD时钟	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4); //GPIOD14复用为定时器4
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;           //GPIOD14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化PD14
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM4 Channe3 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC3

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM14在CCR3上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM14
}
void buzzer_on(int _psc,int _pwm)
{
	TIM_PrescalerConfig(TIM4,_psc,TIM_PSCReloadMode_Immediate);
	TIM_SetCompare3(TIM4,_pwm);
}
#define MAX_PSC                 1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000
uint16_t psc = 0;
uint16_t pwm = MIN_BUZZER_PWM;

void beepLoop()
{
	
//	        pwm++;
//        psc++;
//        
//        if(pwm > MAX_BUZZER_PWM)
//        {
//            pwm = MIN_BUZZER_PWM;
//        }
//        if(psc > MAX_PSC)
//        {
//            psc = 0;
//        }
        buzzer_on(1000, 12000);

	
	
}