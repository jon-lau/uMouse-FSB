#include "user_tasks.h"
#include "string.h"

//Global UART buffers
char tx_buff[TX_BUFF_LEN];
uint8_t Rx_indx;
uint8_t Rx_data[2], Rx_Buffer[RX_BUFF_LEN];

void StartADCTask(void const * argument) {
	HAL_ADC_Start(&hadc1);

	for(;;) {

	}
}

void StartMotorTask(void const * argument) {

	printf("Enable Motors\n\r");


	HAL_GPIO_WritePin(L_PH_GPIO_Port, L_PH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, R_PH_Pin, GPIO_PIN_SET);

	for(;;){

	}

}

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
    vTaskDelayUntil(&xLastWakeTime, 1000);

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
        vTaskDelayUntil(&xLastWakeTime, 50);
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

