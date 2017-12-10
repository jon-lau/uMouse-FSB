#include "spi.h"


void Init_IMU(void)
{
	uint8_t ctrl[2] = {0x11 & 0x7F, 0x44};
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &ctrl, 2, 1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Init(&htim11);
	HAL_TIM_Base_Start_IT(&htim11);
}

void CheckID(void)
{
	//who_am_i
	uint8_t id[2];
	uint8_t who_am_i = 0x0F | 0x80;
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, &who_am_i, (uint8_t *) &id, 2, 1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);
}

float GetAngle(void)
{
	//Yaw
	uint8_t yaw_H[2];
	uint8_t OUTZ_H_G = 0x27 | 0x80;

	uint8_t yaw_L[2];
	uint8_t OUTZ_L_G = 0x26 | 0x80;

	int16_t yaw;

	//Get Angular rate
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, &OUTZ_H_G, (uint8_t *) &yaw_H, 2, 1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, &OUTZ_L_G, (uint8_t *) &yaw_L, 2, 1000);
	HAL_GPIO_WritePin(GPIOC, IMU_CS_Pin, GPIO_PIN_SET);

	yaw = (yaw_H[1] << 8) | (yaw_L[1]);
	if(yaw_H[1] & 0x80 == 0x80)
	{
		yaw = -(~yaw + 1);
	}
	return (yaw*0.01);
}
