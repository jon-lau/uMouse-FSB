#include "QEI.h"

//Timer rolls over at 0xFFFFFFFF ticks


void QEI_Left_Start()
{
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}
void QEI_Right_Start()
{
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void QEI_Velocity_Start()
{
	HAL_TIM_Base_Init(&htim10);
	HAL_TIM_Base_Start_IT(&htim10);
}

void QEI_Left_Reset()
{
	HAL_TIM_Encoder_Stop(&htim5, TIM_CHANNEL_ALL);
	LEFTQEI->CNT = 0;
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}
void QEI_Right_Reset()
{
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	RIGHTQEI->CNT = 0;
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

int QEI_Left_Read()
{
	return -(LEFTQEI->CNT);
}
int QEI_Right_Read()
{
	return RIGHTQEI->CNT;
}

