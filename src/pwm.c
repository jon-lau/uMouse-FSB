#include "pwm.h"

void Left_Motor_Start()
{
	htim3.Init.Prescaler = PWM_TIMER_PRESCALE;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	//1000000 ticks per sec
	//up period later (higher frequency)
	//period = 1000000 ticks per sec / freq
	//set period to 63 for annoying noises
	htim3.Init.Period = 40;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
void Right_Motor_Start()
{
	htim3.Init.Prescaler = PWM_TIMER_PRESCALE;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	//1000000 ticks per sec
	//up period later (higher frequency)
	//period = 1000000 ticks per sec / freq
	//set period to 63 for annoying noises
	htim3.Init.Period = 40;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void Left_Motor_ChangeDuty(float percent)
{
	//percent from -1 to 1
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if(percent > 0)
	{
		if(percent > MAXFORWARD)
		{
			sConfigOC.Pulse = htim3.Init.Period;
		}
		else
		{
			sConfigOC.Pulse = htim3.Init.Period*percent;
		}
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}
	else if(percent < 0)
	{
		if(percent < MAXREVERSE)
		{
			sConfigOC.Pulse = htim3.Init.Period;
		}
		else
		{
			percent = percent * -1;
			sConfigOC.Pulse = htim3.Init.Period*percent;
		}
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}
	else
	{
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}
}
void Right_Motor_ChangeDuty(float percent)
{
	//percent from -1 to 1
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if(percent > 0)
	{
		if(percent > MAXFORWARD)
		{
			sConfigOC.Pulse = htim3.Init.Period;
		}
		else
		{
			sConfigOC.Pulse = htim3.Init.Period*percent;
		}
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	}
	else if(percent < 0)
	{
		if(percent < MAXREVERSE)
		{
			sConfigOC.Pulse = htim3.Init.Period;
		}
		else
		{
			percent = percent * -1;
			sConfigOC.Pulse = htim3.Init.Period*percent;
		}
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	}
	else
	{
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	}
}
