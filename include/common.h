#ifndef COMMON_H
#define COMMON_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define MAX_F_KEYS 3
#define RX_BUFF_LEN 128
#define TOKEN_LEN 56
#define TX_BUFF_LEN 128

extern ADC_HandleTypeDef hadc1;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

extern UART_HandleTypeDef huart1;

extern osThreadId defaultTaskHandle;
extern osThreadId menuTaskHandle;
extern SemaphoreHandle_t xUARTTransferSemaphore;

extern char tx_buff[TX_BUFF_LEN];

#endif
