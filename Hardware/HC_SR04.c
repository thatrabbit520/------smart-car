#include "HC_SR04.h"
#include "Delay.h" 

//超声波模块初始化
void HC_SR04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 开启GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 配置TRIG触发引脚：推挽输出
    GPIO_InitStructure.GPIO_Pin = SR04_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SR04_TRIG_PORT, &GPIO_InitStructure);

    // 配置ECHO接收引脚：浮空输入
    GPIO_InitStructure.GPIO_Pin = SR04_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SR04_ECHO_PORT, &GPIO_InitStructure);

    // 初始状态TRIG拉低
    GPIO_ResetBits(SR04_TRIG_PORT, SR04_TRIG_PIN);
}

//获取超声波测距值
float HC_SR04_GetDistance(void)
{
    uint32_t timeout = 0;
    uint32_t echo_high_time = 0;
    float distance = 0.0f;

    // 发送10us高电平触发信号
    GPIO_SetBits(SR04_TRIG_PORT, SR04_TRIG_PIN);
    Delay_us(10); 
    GPIO_ResetBits(SR04_TRIG_PORT, SR04_TRIG_PIN);

    // 等待ECHO引脚变高
    timeout = 1000000;
    while(GPIO_ReadInputDataBit(SR04_ECHO_PORT, SR04_ECHO_PIN) == RESET)
    {
        timeout--;
        if(timeout == 0) return -1.0f;
    }

    // 等待ECHO引脚变低
    timeout = 1000000;
    while(GPIO_ReadInputDataBit(SR04_ECHO_PORT, SR04_ECHO_PIN) == SET)
    {
        echo_high_time++;
        Delay_us(1); // 每1us计数+1
        timeout--;
        if(timeout == 0) return -1.0f;
    }

    // 计算距离：声速除以2
    // 距离(cm)=高电平时间us*340m/s 除2除10000
    distance = (float)echo_high_time * 0.017f;

    // 范围判断1cm~400cm
    if(distance < 1.0f || distance > 400.0f)
    {
        return -1.0f;
    }

    return distance;
}
