#include "timer.h"

float left_velocity = 0;
float right_velocity = 0;
float angle = 0;

int left_counts = 0;
int left_last_counts = 0;
int right_counts = 0;
int right_last_counts = 0;
float inst_yaw = 0;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM14)
	{
		HAL_IncTick();
	}
	else if (htim->Instance == QEI_VELOCITY_TIM_LABEL)
	{
		left_last_counts = left_counts;
		left_counts = QEI_Left_Read();
		left_velocity = (((float) (left_counts - left_last_counts))/ENCODERCPR) * ENCSAMPLEHZ * DISTPERREV * CMTOM; // m/s
		right_last_counts = right_counts;
		right_counts = QEI_Right_Read();
		right_velocity = (((float) (right_counts - right_last_counts))/ENCODERCPR) * ENCSAMPLEHZ * DISTPERREV * CMTOM; // m/s
	}
	else if (htim->Instance == IMU_TIM_LABEL)
	{
		inst_yaw = GetAngle();
		if(inst_yaw > 5.5)
		{
//			printf("Instantaneous Yaw: %f\n\r", inst_yaw);
			angle += (inst_yaw/100*.7);
//			printf("%.2f\r\n", angle);
		}
		else if(inst_yaw < -9)
		{
//			printf("Instantaneous Yaw: %f\n\r", inst_yaw);
			angle += (inst_yaw/100*0.64);
//			printf("%.2f\r\n", angle);
		}
//		readADC();
//		if(next_move == FORWARD)
//					{
//						hasSideWalls();
//						RWTotal += RightWallPresent;
//						LWTotal += LeftWallPresent;
//						counter++;
//					}
//					else
//					{
//						RightT = RWTotal/counter;
//						LightT = LWTotal/counter;
//						if(RightT >= 0.6) RightWallPresent =1;
//						else RightWallPresent = 0;
//						if(LightT >= 0.6) LeftWallPresent = 1;
//						else LeftWallPresent = 0;
//						counter = RightT = LightT = RWTotal = LWTotal = 0;
//					}


	}
}
