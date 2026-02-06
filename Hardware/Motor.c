#include "Motor.h"

// 初始化GPIO
void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // GPIOB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // IN1-IN4
    GPIO_InitStructure.GPIO_Pin = IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // default setting
    GPIO_ResetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
    GPIO_ResetBits(IN3_PORT, IN3_PIN);
    GPIO_ResetBits(IN4_PORT, IN4_PIN);
}

// TIM3 PWM(PA6/PA7)
void Motor_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // GPIOA TIM3
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // PA6 PA7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // TIM3 setting
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // TIM3_CH1(PA6)PWM
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = PWM_DUTY;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // TIM3_CH2(PA7)PWM
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // TIM3
    TIM_Cmd(TIM3, ENABLE);
}

// forward
void Motor_Forward(void)
{
    //right
    GPIO_SetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
    
    //left
    GPIO_SetBits(IN3_PORT, IN3_PIN);
    GPIO_ResetBits(IN4_PORT, IN4_PIN);
}
// stop
void Motor_Stop(void)
{
    GPIO_ResetBits(IN1_PORT, IN1_PIN);
    GPIO_ResetBits(IN2_PORT, IN2_PIN);
    GPIO_ResetBits(IN3_PORT, IN3_PIN);
    GPIO_ResetBits(IN4_PORT, IN4_PIN);
}

// 设置左右电机PWM速度(0-999对应0%-100%占空比)
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed)
{
    // TIM3_CH1(PA6)右轮,CH2(PA7)左轮
    TIM_SetCompare1(TIM3, right_speed);
    TIM_SetCompare2(TIM3, left_speed);
}
