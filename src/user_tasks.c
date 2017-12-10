#include "user_tasks.h"
#include "string.h"

int start_adc_polling = 0;
int RWTotal = 0;
int LWTotal = 0;
float RightT = 0;
float LightT = 0;
int counter = 0;
int mode = RIGHTWALL;
//Global UART buffers
char tx_buff[TX_BUFF_LEN];
uint8_t Rx_indx;
uint8_t Rx_data[2], Rx_Buffer[RX_BUFF_LEN];

void StartMenuTask(void const * argument)
{
    //Necessary for accurate sleep times
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    //Local UART buffers
    char token[TOKEN_LEN];
    float key_f[MAX_F_KEYS];

    //Setup UART buffers
    memset(tx_buff, 0, sizeof(tx_buff));
    memset(token, 0, sizeof(token));
    memset(key_f, 0, sizeof(key_f));
    memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
    memset(Rx_data, 0, sizeof(Rx_data));

    //Prepare buffers for printf
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    Rx_indx = 0;

    //Wait 1000 msec
    vTaskDelay(1000);


    printf("Startup\r\n");
    //Start ISR for UART reception
    HAL_UART_Receive_IT(&huart1, Rx_data, 1);
    for(;;)
    {
        //Process UART buffer if a full string was received
        if (xSemaphoreTake(xUARTTransferSemaphore, 0x0) == pdTRUE) {
            //Reset UART buffers
            memset(tx_buff, 0, sizeof(tx_buff));
            memset(token, 0, sizeof(token));
            memset(key_f, 0, sizeof(key_f));
            //Process string
            Process_Command(Rx_Buffer, token, key_f);
        }
        //Sleep for 50msec
        vTaskDelay(50);
    }
}

void StartSensorTask(void const * argument){
	HAL_ADC_Start(&hadc1);
	//initialize pins
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,RESET);
	Start_DMA_ADC();
	Init_IMU();
	QEI_Left_Reset();
	QEI_Right_Reset();
	QEI_Velocity_Start();
	for(;;)
	{
		if(start_adc_polling)
		{
			readADC();
//			printf("Right: %0.2f\r\n", R_ADC);
//			printf("Right Front: %0.2f\r\n", RF_ADC);
//			printf("Left Front: %0.2f\r\n", LF_ADC);
//			printf("Left: %0.2f\r\n", L_ADC);
////			printf("Offset: %0.2f \r\n", LF_ADC-RF_ADC);
//			printf("\n");
			if(next_move == FORWARD)
						{
							hasSideWalls();
							RWTotal += RightWallPresent;
							LWTotal += LeftWallPresent;
							counter++;
						}
						else
						{
							RightT = RWTotal/counter;
							LightT = LWTotal/counter;
							if(RightT >= 0.6) RightWallPresent =1;
							else RightWallPresent = 0;
							if(LightT >= 0.6) LeftWallPresent = 1;
							else LeftWallPresent = 0;
							counter = RightT = LightT = RWTotal = LWTotal = 0;
						}

		}
		vTaskDelay(10);
	}
}

void StartPIDControllerTask (void const * argument)
{
	Left_Motor_Start();
	Right_Motor_Start();
	task_pd_on = 1;
	for(;;)
	{
		if(task_pd_on)
		{
			Position_PD();
		}
		Velocity_PID();
		vTaskDelay(10);
	}
}

void StartMovementTask(void const * argument)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET);
	mode = RIGHTWALL;

	readADC();
	while(R_ADC > 5)
	{
		readADC();
		//if you press the button switch modes
		if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)){
			HAL_Delay(1000);
			switch(mode){
			case RIGHTWALL:
				mode = LEFTWALL;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET);
				break;
			case LEFTWALL:
				mode = RIGHTWALL;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET);
				break;
			}

		}

		vTaskDelay(10);
	}
	start_adc_polling = 1;
	HAL_Delay(250);
	move_in_prog = 0;
//	printf("fafa\n\r");
	for(;;)
	{
		hasFrontWall();
		if(!move_in_prog){
			//next_move = FORWARD;
			Determine_Next_Movement();
			printf("Determine Next move: %d\n\r", next_move);
//			printf("next_move: %d", next_move);
			RightWallPresent = 1;
			LeftWallPresent = 1;
			QEI_Left_Reset();
			QEI_Right_Reset();
		}
		//printf("next command: %d\n\r", next_move);

//		printf("next move: %d\n\r", next_move);
		if(!move_in_prog)
			{
				Init_IMU();
				angle = 0;
			}
		switch(next_move)
		{
		case FORWARD:
			if(!move_in_prog)
			{
				QEI_Left_Reset();
				QEI_Right_Reset();
			}
			Move_Forward_Cells(0.95, 0.3);
		break;
		case TURNRIGHT:
			if(!move_in_prog)
			{
				Init_IMU();
				angle = 0;
			}
			Turn_Right(0.23);
			//Change_Target(0,0);
		break;
		case TURNLEFT:
			if(!move_in_prog)
			{
				Init_IMU();
				angle = 0;
			}
			Turn_Left(0.23);
			//Change_Target(0,0);
		break;
		case TURNAROUND:
			if(!move_in_prog)
			{
				Init_IMU();
				angle = 0;
			}
			Turn_Around(0.23); //was 0.4
			//Change_Target(0,0);
		break;
		case STOP:
			Set_Target(0,0);
			break;
		default:

		break;
		}
		hasFrontWall();
		vTaskDelay(10);
	}
}

//UART Rx ISR callback, will load the received string into Rx_Buffer and 
//then release the UART semaphore to be serviced by menu task
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == USART1) {
        if (Rx_indx == 0) {
            memset(Rx_Buffer, 0, RX_BUFF_LEN);
        }
        if (Rx_data[0] != '\r') {
            Rx_Buffer[Rx_indx++] = Rx_data[0];
        }
        else {
            Rx_indx = 0;
            xSemaphoreGiveFromISR(xUARTTransferSemaphore, &xHigherPriorityTaskWoken);
        }
        HAL_UART_Receive_IT(&huart1, Rx_data, 1);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
