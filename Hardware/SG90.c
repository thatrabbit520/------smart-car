#include "SG90.h"

//SG90舵机初始化，配置50Hz PWM输出，开机默认90度
void SG90_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 配置PA0为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = SG90_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SG90_GPIO_PORT, &GPIO_InitStructure);

    // 1MHz计数频率
    // 自动重装载值20000 20ms
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(SG90_TIM, &TIM_TimeBaseStructure);

    // PWM1模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; // 初始值1500 1.5ms高电平 90度正前方
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(SG90_TIM, &TIM_OCInitStructure);

    // 开启预装载，使能定时器
    TIM_OC1PreloadConfig(SG90_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(SG90_TIM, ENABLE);
    TIM_Cmd(SG90_TIM, ENABLE);
}

//设置舵机角度0~180度
void SG90_SetAngle(uint8_t angle)
{
    //角度限幅
    if(angle > 180) angle = 180;
    //角度转PWM比较值
    uint16_t compare_val = 500 + (uint16_t)(angle * 2000.0f / 180.0f);
    TIM_SetCompare1(SG90_TIM, compare_val);
}
