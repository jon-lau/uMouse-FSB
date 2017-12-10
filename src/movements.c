#include "movements.h"

int next_move = 0;
int move_in_prog = 0;
int last_left_counts = 0;
int last_right_counts = 0;

void Determine_Next_Movement(void)
{
//	printf("LEFT WALL PRESENT: %d\n\r", LeftWallPresent);
//	printf("Right WALL PRESENT: %d\n\\r", RightWallPresent);
//	printf("Front WALL PRESENT: %d\n\r", FrontWallPresent);
	if(mode == RIGHTWALL){
		if(RightWallPresent == 0){
			next_move = TURNRIGHT;
		}
		else if(FrontWallPresent == 0){
			next_move = FORWARD;
		}
		else if(LeftWallPresent == 0){
			next_move = TURNLEFT;
		}
		else{
			next_move = TURNAROUND;
		}

		if(turn ==1){
			next_move = FORWARD;
			turn = 0;
		}

	}

	if(mode == LEFTWALL){
			if(LeftWallPresent == 0){
				next_move = TURNLEFT;
			}
			else if(FrontWallPresent == 0){
				next_move = FORWARD;
			}
			else if(RightWallPresent == 0){
				next_move = TURNRIGHT;
			}
			else{
				next_move = TURNAROUND;
			}

			if(turn ==1){
				next_move = FORWARD;
				turn = 0;
			}
	}
}




void Set_Target(float user_l_target, float user_r_target)
{
	user_left_target_vel = user_l_target;
	user_right_target_vel = user_r_target;
}

void Move_Forward_Cells(float cells, float speed)
{
	//add in acceleration controller
	float dist = cells * CELLTOTICKS;
	Set_Target(speed, speed);
	if(fabs(left_counts) > dist || fabs(right_counts) > dist || (L_ADC < 7.55 && R_ADC < 7.55))
	{
		move_in_prog = 0;
		Set_Target(0,0);	//long stop
		next_move = STOP;	//temporary
	}
	else
	{
		move_in_prog = 1;
	}
}

void Turn_Right(float speed)
{
	Change_Target(speed + 0.2, -speed-0.01);
}

void Turn_Left(float speed)
{
	Change_Target(-speed - 0.1, speed + 0.2);
}

void Turn_Around(float speed)
{
	//added 0.2 to keep mouse in center of the cell
	Change_Target(speed + 0.2, -speed- 0.01);
}
