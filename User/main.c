#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
int main(void)
{
	Motor_GPIO_Init();
	Motor_PWM_Init();
	Motor_Forward();
	while (1)
	{
		
	}
}
