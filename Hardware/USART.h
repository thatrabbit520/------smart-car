#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"


extern uint8_t USART1_RX_BUF;   // 接收缓存
extern uint8_t USART1_RX_FLAG;  // 接收完成标志

void USART1_Init(void);         // USART1初始化

#endif
