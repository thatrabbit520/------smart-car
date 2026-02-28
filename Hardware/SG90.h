#ifndef __SG90_H
#define __SG90_H

#include "stm32f10x.h"

// 引脚定义
#define SG90_TIM        TIM2
#define SG90_GPIO_PORT  GPIOA
#define SG90_GPIO_PIN   GPIO_Pin_0

//函数声明
void SG90_Init(void);                  
void SG90_SetAngle(uint8_t angle);     
#endif
