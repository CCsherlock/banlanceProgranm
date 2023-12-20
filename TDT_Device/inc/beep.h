#ifndef _BEEP_H_
#define _BEEP_H_

#include "board.h"

void buzzerPWMInit(u32 arr,u32 psc);
void beepLoop();
#endif