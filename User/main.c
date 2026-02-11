#include "stm32f10x.h"
#include "Motor.h"
#include "LineSensor.h"
#include "LinePosition.h"
#include "stdlib.h"
#include "OLED.h"
#include "USART.h"

// 改进参数
uint16_t base_speed = 350;    // 基础速度，避免电机死区
float p_gain = 85.0f;         // P参数从116.5减小防止转向过猛
float i_gain = 0.8f;          // 减小I避免积分累加的震荡
float d_gain = 2.0f;          // 重点增加D，强化预判超调，压直线高频晃动
int32_t integral = 0;
int32_t integral_limit = 60;   // 再缩小积分限幅，保留微弱积分作用
int8_t last_offset = 0;
#define OFFSET_DEAD_ZONE 2     // 偏差死区，过滤传感器杂波

// 蓝牙模块变量(JDY-31蓝牙)
uint8_t  car_mode = 0;
int32_t  manual_spd = 300;
int32_t  L_man = 0, R_man = 0;

int main(void)
{
    Motor_GPIO_Init();
    Motor_PWM_Init();
    LineSensor_Init();
    OLED_Init();
    USART1_Init();

    while(1)
    {
        // 手机指令
        if(USART1_RX_FLAG)
        {
            USART1_RX_FLAG = 0;
            uint8_t cmd = USART1_RX_BUF;

            if(cmd == 'Q'){
                car_mode = 0;
                OLED_Clear();
            }
            if(cmd == 'E'){
                car_mode = 1;
                OLED_Clear();
            }

            if(car_mode == 1)
            {
                if(cmd == 'W'){ L_man =  manual_spd; R_man =  manual_spd; Motor_Forward(); }
                if(cmd == 'S'){ L_man = -manual_spd; R_man = -manual_spd; Motor_LeftReversal(); Motor_RightReversal(); }
                if(cmd == 'A'){ L_man = manual_spd/2; R_man = manual_spd; Motor_LeftReversal(); }
                if(cmd == 'D'){ L_man = manual_spd; R_man = manual_spd/2; Motor_RightReversal(); }
                if(cmd == 'X'){ L_man = 0; R_man = 0; }
								//速度挡位低，中，高
                if(cmd == '1') manual_spd = 300;
                if(cmd == '2') manual_spd = 500;
                if(cmd == '3') manual_spd = 800;
            }
        }

        //手动指令
        if(car_mode == 1)
        {
            Motor_SetSpeed(L_man, R_man);
            OLED_ShowString(1,1,"Mode:Manual");
            continue;
        }

        int8_t offset = LinePosition_Calc();

        // 丢线处理
        if(offset == -100)
        {
            Motor_SetSpeed(400, 400);  //改为400
            integral = 0;
            last_offset = 0;

            OLED_ShowString(1,1,"LOST LINE   "); // 新增丢线提示
            continue;
        }

        // 过滤传感器杂波
        if(abs(offset) < OFFSET_DEAD_ZONE)
        {
            offset = 0; // 小偏差视为无偏差
        }

        // PID计算(保留微分滤波，再提D强化抑制晃动)
        int32_t p_out = (int32_t)(offset * p_gain);
        int32_t i_out = (int32_t)(integral * i_gain);
        // 微分滤波保留，避免杂波触发D项突变
        int32_t d_out = (int32_t)(((offset - last_offset) * d_gain) * 0.7f);
        int32_t pid_output = p_out + i_out + d_out;

        // 积分条件积累，只在有效时积累
        if(abs(offset) > OFFSET_DEAD_ZONE)
        {
            integral += offset;
            // 积分限幅
            if(integral > integral_limit) integral = integral_limit;
            if(integral < -integral_limit) integral = -integral_limit;
        }

        // 速度计算
        int32_t left_speed = base_speed + pid_output;
        int32_t right_speed = base_speed - pid_output;
        left_speed = (left_speed < 300) ? 300 : (left_speed > 999) ? 999 : left_speed;
        right_speed = (right_speed < 300) ? 300 : (right_speed > 999) ? 999 : right_speed;

        // 显示offset
        OLED_ShowString(1, 1, "Offset:");
        OLED_ShowSignedNum(1, 8, offset, 3);

        // 显示PID值
        OLED_ShowString(2, 1, "PID Out:");
        OLED_ShowSignedNum(2, 9, pid_output, 4);

        // 显示左右轮速度值
        OLED_ShowString(3, 1, "L:");
        OLED_ShowNum(3, 3, left_speed, 3);
        OLED_ShowString(3, 7, "R:");
        OLED_ShowNum(3, 9, right_speed, 3);

        // 显示基础速度和P值
        OLED_ShowString(4, 1, "Base:");
        OLED_ShowNum(4, 6, base_speed, 3);
        OLED_ShowString(4, 10, "P:");
        OLED_ShowNum(4, 12, (uint8_t)p_gain, 2);

        // 转向执行
        if(pid_output < 0)
        {
            Motor_LeftReversal();
            Motor_SetSpeed(left_speed, right_speed);
        }
        else if(pid_output > 0)
        {
            Motor_RightReversal();
            Motor_SetSpeed(left_speed, right_speed);
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
