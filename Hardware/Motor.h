#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#define IN1_PIN    GPIO_Pin_5
#define IN1_PORT   GPIOB
#define IN2_PIN    GPIO_Pin_6
#define IN2_PORT   GPIOB
#define IN3_PIN    GPIO_Pin_7
#define IN3_PORT   GPIOB
#define IN4_PIN    GPIO_Pin_8
#define IN4_PORT   GPIOB

// PWM duty 
#define PWM_DUTY   400

//º¯Êý
void Motor_GPIO_Init(void);
void Motor_PWM_Init(void);
void Motor_Forward(void);
void Motor_SetSpeed(uint16_t left_speed,uint16_t right_speed);
void Motor_Stop(void);
#endif
