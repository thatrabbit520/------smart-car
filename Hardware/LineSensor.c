#include "LineSensor.h"

void LineSensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = LS_X1_PIN | LS_X2_PIN | LS_X3_PIN | LS_X4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


uint8_t LineSensor_Read(void)
{
    uint8_t sensor_data = 0;
    sensor_data |= (GPIO_ReadInputDataBit(LS_X1_PORT, LS_X1_PIN) ? 0 : 1) << 3;  
    sensor_data |= (GPIO_ReadInputDataBit(LS_X2_PORT, LS_X2_PIN) ? 0 : 1) << 2;  
    sensor_data |= (GPIO_ReadInputDataBit(LS_X3_PORT, LS_X3_PIN) ? 0 : 1) << 1;  
    sensor_data |= (GPIO_ReadInputDataBit(LS_X4_PORT, LS_X4_PIN) ? 0 : 1) << 0;  
    return sensor_data;
}
