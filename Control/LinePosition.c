#include "LineSensor.h"  
#include "LinePosition.h"

// X1(最右)=3,X2(右2)=1,X3(左2)=-1,X4(最左)=-3
static const int8_t SENSOR_WEIGHT[] = {3, 1, -1, -3};

int8_t LinePosition_Calc(void)
{
    uint8_t sensor_data = LineSensor_Read();  
    int16_t sum_weight = 0;
    int16_t sum_valid = 0;

    // 计算加权和
    if (sensor_data & 8) // X1最右检测到黑线
    {
        sum_weight += SENSOR_WEIGHT[0];
        sum_valid++;
    }
    if (sensor_data & 4)  // X2右2检测到黑线
    {
        sum_weight += SENSOR_WEIGHT[1];
        sum_valid++;
    }
    if (sensor_data & 2)  // X3左2检测到黑线
    {
        sum_weight += SENSOR_WEIGHT[2];
        sum_valid++;
    }
    if (sensor_data & 1)  // X4最左检测到黑线
    {
        sum_weight += SENSOR_WEIGHT[3];
        sum_valid++;
    }

    // 丢线处理
    if (sum_valid == 0)
    {
        return -100;  // -100为丢线
    }

    return (int8_t)(sum_weight / sum_valid);
}
