#ifndef CALLBACKS_H
#define CALLBACKS_H

#include "common.h"
#include "QEI.h"
#include "pwm.h"
#include "timer.h"
#include "pid.h"
#include "dma-adc.h"
#include "spi.h"

extern int task_pd_on;
extern int velocity_control_on;	//can possibly get rid of this and tie task_pd together
extern int pd_mode;
extern float left_target_vel;
extern float user_left_target_vel;
extern float right_target_vel;
extern float user_right_target_vel;

#define HOLD_POSITION 0
#define MOVE_TO_POSITION 1
#define VELOCITY 2

//Callbacks
void Help_Callback(int num_keys, float *keys);
void ADC_Callback(int num_keys, float *keys);
void SPI_Callback(int num_keys, float *keys);
void Encoder_Callback(int num_keys, float *keys);
void Motor_Callback(int num_keys, float *keys);
void PD_Callback(int num_keys, float *keys);

#endif
