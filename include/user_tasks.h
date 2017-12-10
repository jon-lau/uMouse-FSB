#ifndef USER_TASKS_H
#define USER_TASKS_H

#include "common.h"
#include "pid.h"
#include "QEI.h"
#include "pwm.h"
#include "dma-adc.h"
#include "spi.h"
#include "callbacks.h"
#include "movements.h"

extern int RWTotal;
extern int LWTotal;
extern float RightT;
extern float LightT;
extern int counter;
extern int mode;

void StartMenuTask(void const * argument);
void StartSensorTask(void const * argument);
void StartPIDControllerTask (void const * argument);
void StartMovementTask(void const * argument);

#endif
