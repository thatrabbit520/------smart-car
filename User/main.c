#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "LineSensor.h" 


#define LED_PIN    GPIO_Pin_0
#define LED_PORT   GPIOA

// LED
void LED_Init1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = LED_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStruct);
    
    GPIO_ResetBits(LED_PORT, LED_PIN);
}

int main(void)
{
    uint8_t sensor_data = 0; 
    
    
    LineSensor_Init();
    LED_Init1();

    while(1)
    {
        // 读取传感器数据
        sensor_data = LineSensor_Read();
        
        // 判断是否检测到黑线
        if(sensor_data != 0)
        {
            GPIO_SetBits(LED_PORT, LED_PIN); // 有黑线 LED亮
        }
        else
        {
            GPIO_ResetBits(LED_PORT, LED_PIN); // 反之 灭
        }
    }
}
