#include "LineSensor.h"
//四路循迹模块初始化
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

    // 读取每个引脚，并对输出值取反（因为四路循迹模块检测到黑线返回0）
    // 最左探头P2对应X1引脚，最低位
    sensor_data |= (GPIO_ReadInputDataBit(LS_X1_PORT, LS_X1_PIN) ? 0 : 1) << 0;
    // 左2探头对应X2，第二位
    sensor_data |= (GPIO_ReadInputDataBit(LS_X2_PORT, LS_X2_PIN) ? 0 : 1) << 1;
    // 右2  P3  X3      以此类推
    sensor_data |= (GPIO_ReadInputDataBit(LS_X3_PORT, LS_X3_PIN) ? 0 : 1) << 2;
    // 最右 P4  X4      以此类推
    sensor_data |= (GPIO_ReadInputDataBit(LS_X4_PORT, LS_X4_PIN) ? 0 : 1) << 3;

    return sensor_data;
}
