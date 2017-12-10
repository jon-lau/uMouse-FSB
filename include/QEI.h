#ifndef QEI_H_
#define QEI_H_
#include "common.h"
#include "main.h"

//encoder 1024 ticks per rev
//gear ratio 12:40 (0.3)
//wheel radius 2.45 cm
//wheel rotation 7.697 cm (pi*wheel radius)

#define LEFTQEI			TIM5
#define RIGHTQEI		TIM2
#define ENCODERCPR		1024
#define ENCSAMPLEHZ		1000
#define DISTPERREV		2.309
#define CMTOM			0.01
#define CELLTOTICKS 	7982.68*2

void QEI_Left_Start();
void QEI_Right_Start();
void QEI_Velocity_Start();

void QEI_Left_Reset();
void QEI_Right_Reset();

int QEI_Left_Read();
int QEI_Right_Read();


#endif
