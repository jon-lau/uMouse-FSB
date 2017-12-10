#ifndef TIMER_H_
#define TIMER_H_

#include "QEI.h"
#include "spi.h"

#define QEI_VELOCITY_TIM_LABEL TIM10
#define IMU_TIM_LABEL TIM11

extern int left_counts;
extern int right_counts;
extern float left_velocity;
extern float right_velocity;
extern float angle;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
