#ifndef COMMON_H
#define COMMON_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define MAX_F_KEYS 3
#define RX_BUFF_LEN 128
#define TOKEN_LEN 56
#define TX_BUFF_LEN 128

ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;	//Right Encoder Timer
TIM_HandleTypeDef htim3;	//Motor Timer
TIM_HandleTypeDef htim5;	//Left Encoder Timer
TIM_HandleTypeDef htim10;	//Encoder Position and Velocity Timer
TIM_HandleTypeDef htim11;	//IMU Angle Poller

UART_HandleTypeDef huart1;

extern DMA_HandleTypeDef hdma_adc1;
extern osThreadId defaultTaskHandle;
extern osThreadId menuTaskHandle;
extern osThreadId sensorTaskHandle;
extern osThreadId movementTaskHandle;
extern osThreadId PIDControllerTaskHandle;
extern SemaphoreHandle_t xUARTTransferSemaphore;

extern char tx_buff[TX_BUFF_LEN];

#endif
