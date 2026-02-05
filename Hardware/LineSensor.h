#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#include "stm32f10x.h"

#define LS_X1_PIN   GPIO_Pin_0
#define LS_X2_PIN   GPIO_Pin_1
#define LS_X3_PIN   GPIO_Pin_10
#define LS_X4_PIN   GPIO_Pin_11
#define LS_X1_PORT  GPIOB
#define LS_X2_PORT  GPIOB
#define LS_X3_PORT  GPIOB
#define LS_X4_PORT  GPIOB

void LineSensor_Init(void);
uint8_t LineSensor_Read(void);

#endif
