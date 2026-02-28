#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "stm32f10x.h"

// 超声波引脚定义
#define SR04_TRIG_PORT  GPIOA
#define SR04_TRIG_PIN   GPIO_Pin_1
#define SR04_ECHO_PORT  GPIOA
#define SR04_ECHO_PIN   GPIO_Pin_2

// 函数声明
void HC_SR04_Init(void);                  
float HC_SR04_GetDistance(void);          

#endif
