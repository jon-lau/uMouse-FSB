#ifndef PID_H_
#define PID_H_
#include "common.h"
#include "main.h"
#include <math.h>
#include "pwm.h"
#include "QEI.h"
#include "timer.h"
#include "callbacks.h"
#include "dma-adc.h"
#include "movements.h"

//Left Motor
#define LEFTP 0.4
#define LEFTI 0.5
#define LEFTD 0.125

//Right Motor
#define RIGHTP 1.3
#define RIGHTI 0.5
#define RIGHTD 0.2375

//PID Vars
#define leftMiddle 7.4
#define rightMiddle 7
#define offsetPID 0.4

extern float testP;
extern float testI;
extern float testD;

extern float L_errorI_v;
extern float R_errorI_v;

extern int turn;

void Change_Target(float l_target, float r_target);
void Velocity_PID(void);
void Position_PD(void);
void Rotation_Controller(int select);


#endif
