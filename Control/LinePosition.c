#include "LineSensor.h"  
#include "LinePosition.h"

int8_t LinePosition_Calc(void)
{
    uint8_t sensor_data = LineSensor_Read();
    int16_t sum_weight = 0;
    int16_t sum_valid = 0;

    // 权重顺序:最右(P4)右2(P3)左2(P1)最左(P2)
    static const int8_t SENSOR_WEIGHT[] = {3, 1, -1, -3};

    // 最右 最高位
    if (sensor_data & 8)
    {
        sum_weight += SENSOR_WEIGHT[0];
        sum_valid++;
    }
    // 右2  第3位
    if (sensor_data & 4)
    {
        sum_weight += SENSOR_WEIGHT[1];
        sum_valid++;
    }
    // 左2  第2位
    if (sensor_data & 2)
    {
        sum_weight += SENSOR_WEIGHT[2];
        sum_valid++;
    }
    // 最左  最低位
    if (sensor_data & 1)
    {
        sum_weight += SENSOR_WEIGHT[3];
        sum_valid++;
    }

    // 丢线处理
    if (sum_valid == 0)
    {
        return -100;
    }

    return (int8_t)(sum_weight / sum_valid);
}
