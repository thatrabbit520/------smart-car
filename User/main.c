#include "stm32f10x.h"
#include "Motor.h"
#include "LineSensor.h"
#include "LinePosition.h"

// 完善PID各项参数，实现完整PID控制
uint16_t base_speed = 350;    // 基础速度，避免电机死区
float p_gain = 85.0f;         // P参数从116.5减小防止转向过猛
float i_gain = 2.5f;          // I参数减小防止积分累加加剧震荡
float d_gain = 1.2f;          // D参数增加提前抑制转向超调
int32_t integral = 0;         // 积分积累值
int32_t integral_limit = 120; // 积分上限，防止饱和
int8_t last_offset = 0;       // 上一次偏差值，微分运算

int main(void)
{
    Motor_GPIO_Init();
    Motor_PWM_Init();
    LineSensor_Init();

    while(1)
    {
        // 获取当前轨迹差
        int8_t offset = LinePosition_Calc();

        // 丢线处理
        if(offset == -100)
        {
            Motor_SetSpeed(350, 350); 
            integral = 0;
            last_offset = 0;
            continue;
        }

        // PID计算
        int32_t p_out = (int32_t)(offset * p_gain);
        int32_t i_out = (int32_t)(integral * i_gain);
        int32_t d_out = (int32_t)((offset - last_offset) * d_gain);

        // 积分防饱和处理
        integral += offset;
        if(integral > integral_limit) integral = integral_limit;
        if(integral < -integral_limit) integral = -integral_limit;

        // PID总输出
        int32_t pid_output = p_out + i_out + d_out;

        // 速度计算
        int32_t left_speed = base_speed + pid_output;
        int32_t right_speed = base_speed - pid_output;

        // 速度限幅
        left_speed = (left_speed < 300) ? 300 : (left_speed > 999) ? 999 : left_speed;
        right_speed = (right_speed < 300) ? 300 : (right_speed > 999) ? 999 : right_speed;

        // 转向执行
        if(left_speed < 300) 
        {
            Motor_LeftReversal();
            Motor_SetSpeed(300, right_speed); 
        }
        else if(right_speed < 300)
        {
            Motor_RightReversal();
            Motor_SetSpeed(left_speed, 300); 
        }
        else
        {
            Motor_Forward();
            Motor_SetSpeed(left_speed, right_speed);
        }

        // 更新上次偏差
        last_offset = offset;
    }
}
