#ifndef _BEEP_H_
#define _BEEP_H_

#include "board.h"
#define MAX_PSC 1000

#define MAX_BUZZER_PWM 20000
#define MIN_BUZZER_PWM 10000

#define HIGH_TONE 1
#define MIDDLE_TONE 2
#define LOW_TONE 3
class Beep
{
private:
    /* data */
    enum BeepState
    {
        SILENT = 0,
        RING
    };
    BeepState beepState = SILENT;
    uint16_t _psc = 3;
    uint16_t _pwm = MIN_BUZZER_PWM+2000;
    uint32_t ringTime = 100;
    float runingTime,runingTime_last;
    uint8_t ringLoopTimes;
    bool ringLoopReset = true;
public:
    Beep(/* args */);
    void beepOn();
    void beepOff();
    void beepToggle();
    void beepAlarm(uint8_t times);
    void setBeepTone(uint16_t tone);
};
void buzzerPWMInit(u32 arr, u32 psc);
extern Beep beep;
#endif