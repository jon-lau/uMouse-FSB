#ifndef PWM_H_
#define PWM_H_
#include "common.h"
#include "main.h"

#define TIMER_CLK_SPEED 84000000
#define PWM_TIMER_PRESCALE 83
#define MAXFORWARD 1
#define MAXREVERSE -1

#define INITDUTY 0.5

void Left_Motor_Start();
void Right_Motor_Start();
void Left_Motor_ChangeDuty(float percent);
void Right_Motor_ChangeDuty(float percent);

#endif
