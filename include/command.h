#ifndef COMMAND_H
#define COMMAND_H

#include "common.h"
#include "LSM9DS1.h"

#define CMD_ERR_INVALID_VAL -1
#define CMD_ERR_MISSING_ARG -2
#define CMD_ERR_TOO_MANY_ARG -3

#define NUM_COMMANDS (sizeof(Command_List)/sizeof(Command_List[0]))

uint8_t Parse_Entry_Float(uint8_t *input_string, uint8_t *token, float *keys, uint8_t max_keys);
int ftoi_h(float a);
int ftoi_l(float a);
float atof_f(uint8_t *in);
uint8_t* ftoa(float a);

int Process_Command(uint8_t *input_string, uint8_t *token, float *keys);

typedef struct {
    char name[TOKEN_LEN];
    int (*callback)(int, float*);
    int num_args;
} Command_Item;

//Callbacks
void Help_Callback(int num_keys, float *keys);
void IMU_Accel_Callback(int num_keys, float *keys);
void Motor_Callback(int num_keys, float *keys);
void IMU_Callback(int num_keys, float *keys);
void GPIO_Callback(int num_keys, float *keys);
void Switch_Callback(int num_keys, float *keys);
void ADC_Callback(int num_keys, float *keys);

#endif
