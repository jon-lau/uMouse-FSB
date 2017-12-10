#include "pid.h"

float L_errorP_v = 0;
float L_old_errorP_v = 0;
float L_errorD_v = 0;
float L_errorI_v = 0;
float L_totalError_v = 0;

float R_errorP_v = 0;
float R_old_errorP_v = 0;
float R_errorI_v = 0;
float R_errorD_v = 0;
float R_totalError_v = 0;

float testP = 0;
float testI = 0;
float testD = 0;
float errorP = 0;
float errorI = 0;
float oldErrorP = 0;
float errorD;
float totalError = 0;
float Pk = 0.005;    //P initial = 0.03 0.04
float Ik = 0.000005;
float Dk = 0.001; //D initial = 0.000000005 000000020 0.002
int turn = 0;

void Change_Target(float l_target, float r_target)
{
	L_errorI_v = 0; //I has to be reset to 0 every time setpoint changes
	R_errorI_v = 0; //I has to be reset to 0 every time setpoint changes
	left_target_vel = l_target;
	right_target_vel = r_target;
}

void Velocity_PID(void)
{

	//need to add deadband for stopped
	L_errorP_v = left_target_vel - left_velocity;
	L_errorI_v = L_errorI_v + L_errorP_v;
	L_errorD_v = L_errorP_v - L_old_errorP_v;

	L_totalError_v = LEFTP * L_errorP_v + LEFTI * L_errorI_v + LEFTD * L_errorD_v;

	L_old_errorP_v = L_errorP_v;

	if(left_target_vel > 0)
	{
		if(L_totalError_v > 1)
		{
			L_totalError_v = 1;
		}
		else if(L_totalError_v < 0)
		{
			L_totalError_v = 0;
		}
	}
	else if(left_target_vel < 0)
	{
		if(L_totalError_v < -1)
		{
			L_totalError_v = -1;
		}
		else if(L_totalError_v > 0)
		{
			L_totalError_v = 0;
		}
	}
	Left_Motor_ChangeDuty(L_totalError_v);


	//need to add deadband for stopped
	R_errorP_v = right_target_vel - right_velocity;
	R_errorI_v = R_errorI_v + R_errorP_v;
	R_errorD_v = R_errorP_v - R_old_errorP_v;

	R_totalError_v = RIGHTP * R_errorP_v + RIGHTI * R_errorI_v + RIGHTD * R_errorD_v;

	R_old_errorP_v = R_errorP_v;

	if(right_target_vel > 0)
	{
		if(R_totalError_v > 1)
		{
			R_totalError_v = 1;
		}
		else if(R_totalError_v < 0)
		{
			R_totalError_v = 0;
		}
	}
	else if(right_target_vel < 0)
	{
		if(R_totalError_v < -1)
		{
			R_totalError_v = -1;
		}
		else if(R_totalError_v > 0)
		{
			R_totalError_v = 0;
		}
	}

	Right_Motor_ChangeDuty(R_totalError_v);
}

void Position_PD(void)
{
	if(next_move == FORWARD){
//		printf("Right Front: %0.2f\r\n", RF_ADC);
//		printf("Left Front: %0.2f\r\n", LF_ADC);
//			if(RF_ADC < 9.5 && LF_ADC < 8.5){ //if has both walls
//			errorP = LF_ADC - RF_ADC + offsetPID;
//			printf("correcting off both walls\n\r");
//			errorD = errorP - oldErrorP;
//			errorI = errorI + errorP;
//			L_errorI_v = L_errorI_v + L_errorP_v;

		if(RF_ADC < 9.5){ // if only has right wall
			errorP = 2 *(rightMiddle - RF_ADC); //was 0.0005 .0000005
			errorI = errorI + errorP;
			errorD = errorP - oldErrorP;
//			printf("correcting off right wall\n\r");
		}
		else if(LF_ADC < 8.5){ // if only has left wall
			errorP = 2*(LF_ADC- leftMiddle); //was left - LfADC
			errorI = errorI + errorP;
//			printf("Correcting off left wall\n\r");
			errorD = errorP - oldErrorP;
		}
		else{ //has no walls, Pray
//			printf("in case 4\n\r");
			errorP = 0;
			oldErrorP = errorP;
		}
		totalError = Pk * errorP + Ik * errorI + Dk * errorD;
		oldErrorP = errorP;
//		errorP = 2 * (leftMiddle - LF_ADC);
//		printf("LeftVel: %0.2f\n\r", errorP);
//		printf("RightVel: %0.2f\n\r", errorP);
//		printf("\n\r");

		Change_Target(user_left_target_vel - totalError, user_right_target_vel + totalError);
	}
	else
	{

		if(!move_in_prog)
		{
			Init_IMU();
			angle = 0;
		}
		switch(next_move)
		{
		case TURNRIGHT:	//Turn right
			Rotation_Controller(TURNRIGHT);
		break;
		case TURNLEFT:	//Turn left
			Rotation_Controller(TURNLEFT);
		break;
		case TURNAROUND:	//Turn around
			Rotation_Controller(TURNAROUND);
		break;
		case STOP:
			Set_Target(0,0);
			break;
		default:

		break;
		}
	}
	}


void Rotation_Controller(int select){
	switch(select){
		case TURNRIGHT:
			if(angle > -590)
			{
//				Change_Target(0.4, -0.4);
				move_in_prog = 1;
			}
			else
			{
				Set_Target(0,0);
				RightWallPresent = 1;
				LeftWallPresent = 1;
				QEI_Left_Reset();
				QEI_Right_Reset();
				angle = 0;
				move_in_prog = 0;
				turn = 1;
				next_move = FORWARD;

				//Move_Forward_Cells(1, 0.4);
				//Change_Target(0,0);
			}
		break;

		case TURNLEFT:
			if(angle < 440){
//				Change_Target(-0.4, 0.4);
				move_in_prog = 1;
			}
			else
			{
				Set_Target(0,0);
				RightWallPresent = 1;
				LeftWallPresent = 1;
				QEI_Left_Reset();
				QEI_Right_Reset();
				angle = 0;
				move_in_prog = 0;
				turn = 1;
				next_move = FORWARD;
				//Change_Target(0,0);
			}
		break;

		case TURNAROUND:
			if(angle > -1285)
			{
//				Change_Target(0.4, -0.4);
				move_in_prog = 1;
			}
			else
			{
				Set_Target(0,0);
				RightWallPresent = 1;
				LeftWallPresent = 1;
				QEI_Left_Reset();
				QEI_Right_Reset();
				angle = 0;
				move_in_prog = 0;
				turn = 1;
				next_move = FORWARD;
			//	Change_Target(0,0);
			}
		break;
	}

}
