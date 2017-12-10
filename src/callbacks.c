#include "callbacks.h"

int velocity_control_on = 0;
int task_pd_on = 0;
int pd_mode = 0;
int movement_given = 0;
float left_target_vel = 0;
float user_left_target_vel = 0;
float right_target_vel = 0;
float user_right_target_vel = 0;

void Help_Callback(int num_keys, float *keys)
{
   /* Ignored but will print out all other available commands */
}

void ADC_Callback(int num_keys, float *keys)
{
	//Read ADC VALUES
	readADC();

//	printf("Right: %0.2f\r\n", R_ADC);
//	printf("Right Front: %0.2f\r\n", RF_ADC);
//	printf("Left Front: %0.2f\r\n", LF_ADC);
//	printf("Left: %0.2f\r\n", L_ADC);
//	printf("\n");
}

void SPI_Callback(int num_keys, float *keys)
{
	GetAngle();
	printf("Angle: %.2f\r\n", angle);
}

void Encoder_Callback(int num_keys, float *keys)
{
	switch((int) keys[0])
	{
	case 0:
		printf("Left Encoder Counts: %d\r\n", QEI_Left_Read());
		printf("Right Encoder Counts: %d\r\n", QEI_Right_Read());
		break;
	case 1:
		QEI_Left_Reset();
		QEI_Right_Reset();
		break;
	case 2:
		printf("Left Velocity: %.2f\r\n", left_velocity);
		printf("Right Velocity: %.2f\r\n", right_velocity);
		break;
	default:
		break;
	}
}

void PD_Callback(int num_keys, float *keys)
{
	switch((int) keys[0])
	{
	case 0:
		//fomat: 0 x x
		velocity_control_on = 0;
		task_pd_on = 0;
		Left_Motor_ChangeDuty(0);
		Right_Motor_ChangeDuty(0);
		printf("PD off\r\n");
		break;
	case 1:
		//format: 1 (dist. in m) (speed in m/s)
		task_pd_on = 1;
		Change_Target((float) keys[1], (float) keys[1]);
		pd_mode = MOVE_TO_POSITION;
		printf("Postion PD Mode\r\n");
		break;
	case 2:
		velocity_control_on = 1;
		Change_Target((float) keys[1], (float) keys[1]);
		printf("PID Velocity Mode\r\n");
		break;

/*
	case 3:
		testP = (float) keys[1];
		printf("P is now %.2f\r\n", testP);
		break;
	case 4:
		testI = (float) keys[1];
		printf("I is now %.2f\r\n", testI);
		break;
	case 5:
		testD = (float) keys[1];
		printf("D is now %.2f\r\n", testD);
		break;
*/
	default:
		break;
	}
}

void Motor_Callback(int num_keys, float *keys)
{
	Left_Motor_ChangeDuty((float) keys[0]);
	Right_Motor_ChangeDuty((float) keys[0]);
}
