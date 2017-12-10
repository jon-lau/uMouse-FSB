#ifndef MOVEMENTS_H_
#define MOVEMENTS_H_

#include "common.h"
#include "main.h"
#include <math.h>
#include "pwm.h"
#include "QEI.h"
#include "timer.h"
#include "callbacks.h"
#include "dma-adc.h"
#include "pid.h"
#include "user_tasks.h"

#define FORWARD 0
#define TURNRIGHT 1
#define TURNLEFT 2
#define TURNAROUND 3
#define STOP 4
#define RIGHTWALL 100
#define LEFTWALL 200



extern int next_move;
extern int move_in_prog;


void Determine_Next_Movement(void);
void Set_Target(float user_l_target, float user_r_target);
void Move_Forward_Cells(float cells, float speed);
void Turn_Right(float speed);
void Turn_Left(float speed);
void Turn_Around(float speed);


#endif
