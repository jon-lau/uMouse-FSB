#include "command.h"

//List of menu items, name is used for string matching to callbacks
Command_Item Command_List[] =   {
           {.name="help", .callback=Help_Callback, .num_args=0},
		   {.name="accel", .callback=IMU_Accel_Callback, .num_args=0},
		   {.name="motor", .callback=Motor_Callback, .num_args=0},
		   {.name="imu", .callback=IMU_Callback, .num_args=0},
		   {.name="gpio", .callback=GPIO_Callback, .num_args=0},
		   {.name="switch", .callback=Switch_Callback, .num_args=0},
		   {.name="adc", .callback=ADC_Callback, .num_args=0},
                                };

// Callbacks can go here or in another file 
void Help_Callback(int num_keys, float *keys)
{
   /* Ignored but will print out all other available commands */ 
}

void ADC_Callback(int num_keys, float *keys) {
	int sum = 0;
//	for(;;){
//		for(int j = 0; j < 4; j++ ){
//				switch(j){
//					case 0:  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
//					case 1:  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
//					case 2:  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
//					case 3:  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
//				}
//			for(int i = 0; i < 10; i++){
//				if(HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) {
//					sum += HAL_ADC_GetValue(&hadc1);
//					//printf("Data[]: %ld\n\r", HAL_ADC_GetValue(&hadc1));
//				}
//			}
//			sum = sum/10;
//			printf("Data[%d]: %d\n\r", j , sum);
//
//			switch(j){
//				case 0:  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
//				case 1:  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
//				case 2:  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
//				case 3:  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
//			}
//
//		}
//
//		printf("\n");


			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

			while(1){
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
			}


printf("THIS IS A TEST");


//	}


}

void Switch_Callback(int num_keys, float *keys) {
	printf("Switch 1: %d\n\r", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));
	printf("Switch 2: %d\n\r", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5));
}

void GPIO_Callback(int num_keys, float *keys) {

	printf("Left Enable: %d\n\r", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
	printf("Right Enable: %d\n\r", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));

}

void IMU_Callback(int num_keys, float *keys) {
	resetAndCheck();
}

void Motor_Callback(int num_keys, float *keys) {

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
}

void IMU_Accel_Callback(int num_keys, float *keys) {
	printf("x:%ld\n\r", (uint32_t)accelData.x);
	printf("y:%ld\n\r", (uint32_t)accelData.y);
	printf("z:%ld\n\r", (uint32_t)accelData.z);
}

//Receives input string and parses it to see which callback to use
int Process_Command(uint8_t *input_string, uint8_t *token, float *keys)
{
    int i;
    int num_keys = Parse_Entry_Float(input_string, token, keys, MAX_F_KEYS);
    int err_code;
    for (i = 0; i < NUM_COMMANDS; i++) {
        if (strcmp(token, Command_List[i].name) == 0) {
            if (i == 0) {
                for (i = 1; i < NUM_COMMANDS; i++) {
                    printf("%s\targs:%d\r\n", Command_List[i].name, Command_List[i].num_args);
                }
                i = 0;
                break;
            }
            else {
                if (Command_List[i].num_args > num_keys) {
                    err_code = CMD_ERR_MISSING_ARG;
                }
                else if (Command_List[i].num_args < num_keys) {
                    err_code = CMD_ERR_TOO_MANY_ARG;
                }
                else {
                    err_code = Command_List[i].callback(num_keys, keys);
                }
                switch (err_code) {
                    case CMD_ERR_INVALID_VAL:
                        sprintf(tx_buff, "Invalid argument value\r\n");
                        break;
                    case CMD_ERR_MISSING_ARG:
                        sprintf(tx_buff, "Missing argument(s)\r\n");
                        break;
                    case CMD_ERR_TOO_MANY_ARG:
                        sprintf(tx_buff, "Too many arguments\r\n");
                        break;
                }
                if (strlen(tx_buff) < 1) {
                    sprintf(tx_buff, "%s\r\n", Command_List[i].name);
                }
                printf("%s", tx_buff);
                break;
            }
        }
    }
    if (i >= NUM_COMMANDS) {
        printf("Invalid: [%s]\r\n", input_string);
    }
}

//Parses input string into a command name and an array of floating point values
uint8_t Parse_Entry_Float(uint8_t *input_string, uint8_t *token, float *keys, uint8_t max_keys) 
{
    uint8_t num_keys = 0;
    uint8_t *pch;
    pch = (uint8_t*)strchr((char*)input_string, ' ');
    if (pch == NULL) {
        strncpy((char*)token, (char*)input_string, TOKEN_LEN);
    }
    else {
        while (pch != NULL) {
            //First found
            if (num_keys == 0) {
                strncpy((char*)token, (char*)input_string, pch-input_string);
                keys[0] = atof_f(pch+1);
            }
            else if (num_keys == max_keys)
                break;
            else {
                keys[num_keys] = atof_f(pch+1);
            }
            ++num_keys;
            pch = (uint8_t*)strchr((char*)pch+1, ' ');
        }
    }
    return num_keys;
}

float atof_f(uint8_t *in)
{
    float base = 0.0;
    int dec_flag = 0;
    float neg_flag = 1.0;
    while (((*in >= '0') && (*in <= '9')) || (*in == '.') || (*in == '-')) {
        if (((*in >= '0') && (*in <= '9'))) {
            if (!dec_flag) {
                base *= 10.0;
                base += (float)((*in) - '0');
            }
            else {
                base += (float)((*in)-'0')/(dec_flag);
                dec_flag *= 10.0;
            }
        }
        else if (*in == '-') {
            neg_flag = -1.0;
        }
        else {
            dec_flag = 10;
        }
        in++;
    }
    return base*neg_flag;
}

int ftoi_h(float a)
{
    return (int)a;
}

int ftoi_l(float a) 
{
    if (a < 0.0) {
        a = -a;
    }
    return (int)((a-(int)a)*100);
}
