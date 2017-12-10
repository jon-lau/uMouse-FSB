#ifndef SPI_H_
#define SPI_H_

#include "common.h"
#include "main.h"
#include "callbacks.h"

void Init_IMU(void);
void CheckID(void);
float GetAngle(void);

#endif
