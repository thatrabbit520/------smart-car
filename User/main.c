#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"        
#include "LineSensor.h"   
#include "LinePosition.h" 

int main(void)
{
    // 初始化各模块
    Motor_GPIO_Init();   
    Motor_PWM_Init();    
    LineSensor_Init();   
    //PID控制模块（待优化）
    // P参数
    uint16_t base_speed = 300;  // 初始速度
    float p_gain = 150.0f;       // 开始时调为15.0发现根本不够用，暂定为150.0

    while(1)
    {
        //获取位置偏差
        int8_t offset = LinePosition_Calc();

        // 丢线情况
        if(offset == -100)
        {
            Motor_SetSpeed(300,300);
            continue; // 丢线不合理，待优化
        }

        //P参数调整左右轮速差
        int16_t left_speed = base_speed + offset * p_gain;
        int16_t right_speed = base_speed - offset * p_gain;

        //速度范围限定
        left_speed = (left_speed < 0) ? 0 : (left_speed > 999) ? 999 : left_speed;
        right_speed = (right_speed < 0) ? 0 : (right_speed > 999) ? 999 : right_speed;

        //前进指令
        Motor_SetSpeed(left_speed, right_speed);
        Motor_Forward();
    }
}
