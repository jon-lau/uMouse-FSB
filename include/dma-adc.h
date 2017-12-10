#ifndef DMA_ADC_H_
#define DMA_ADC_H_
#include "common.h"
#include "callbacks.h"

#define ADC_BUFF_SIZE 	44
#define NUM_ADC_CH		4
#define HASLEFTWALL		8
#define HASRIGHTWALL	8
#define OFFSET			0.9


extern float R_ADC;
extern float RF_ADC;
extern float LF_ADC;
extern float L_ADC;
extern int LeftWallPresent;
extern int RightWallPresent;
extern int FrontWallPresent;

void Start_DMA_ADC(void);
void readADC(void);
void hasSideWalls(void);
void hasFrontWall(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
float Mean_Filter_ADC_Channel(uint32_t ch);

#endif
