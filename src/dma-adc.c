#include "dma-adc.h"

//Global ADC Values
uint32_t ADC_Values[ADC_BUFF_SIZE];
uint32_t Filtered_ADC_Values[NUM_ADC_CH];
float R_ADC;
float RF_ADC;
float LF_ADC;
float L_ADC;
int LeftWallPresent = 1;
int RightWallPresent = 1;
int FrontWallPresent = 0;

void readADC(void)
{
	for(int i = 0; i < NUM_ADC_CH; i++){
		switch(i){
			case 0:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,RESET);
				HAL_Delay(10);
				L_ADC = (Filtered_ADC_Values[2] - 3531.0) / (-352.0);
				break;
			case 1:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,RESET);
				HAL_Delay(10);
				R_ADC = (Filtered_ADC_Values[1] - 4382.0) / (-393.0);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,RESET);
				HAL_Delay(10);
				RF_ADC = (Filtered_ADC_Values[0]-1930.08)/-191.08;
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,RESET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,RESET);
				HAL_Delay(10);
				LF_ADC = (Filtered_ADC_Values[3]-2464.59)/-257.25;
				break;
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12,RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,RESET);
	}
}

void hasSideWalls(void)
{
	readADC();
	if(RightWallPresent)
	{
		if(RF_ADC > 9.7)
		{
//			printf("lost right wall\r\n");
			RightWallPresent = 0;
		}
		else
		{
			RightWallPresent = 1;
		}
	}
	if(LeftWallPresent)
	{
		if(LF_ADC > 9.25)
		{
//			printf("lost left wall\r\n");
			LeftWallPresent = 0;
		}
		else
		{
			LeftWallPresent = 1;
		}
	}
}

void hasFrontWall(void)
{
	if(L_ADC < 9.6 && R_ADC < 10.6) //replace with macro later
	{
		FrontWallPresent = 1;
	}
	else
	{
		FrontWallPresent = 0;
	}

}

void Start_DMA_ADC(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Values, ADC_BUFF_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	Filtered_ADC_Values[0] = Mean_Filter_ADC_Channel(0);
	Filtered_ADC_Values[1] = Mean_Filter_ADC_Channel(1);
	Filtered_ADC_Values[2] = Mean_Filter_ADC_Channel(2);
	Filtered_ADC_Values[3] = Mean_Filter_ADC_Channel(3);
}

float Mean_Filter_ADC_Channel(uint32_t ch)
{
    uint32_t i;
    float sample = 0;
    for (i = 0; i < 9; i++) {
        sample += ADC_Values[ADC_BUFF_SIZE - NUM_ADC_CH * (i + 1) + ch];
    }
    return sample/9;
}
