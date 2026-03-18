/*
 * @Author: Xiang xin wang wxinxiang8@gmail.com
 * @Date: 2026-03-18 12:57:08
 * @LastEditors: Xiang xin wang wxinxiang8@gmail.com
 * @LastEditTime: 2026-03-18 18:43:05
 * @FilePath: \MDK-ARMd:\shrimp_machine\shrimp_machine\Core\Src\dc_motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "dc_motor.h"
#include "main.h"
#include "tim.h"

#define DC_MOTOR_MAX_SPEED_RPM   1800.0f
#define DC_MOTOR_MAX_DUTY_RATIO  0.98f

void DC_Motor_Init(void)
{
    /* 启动定时器 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void DC_Motor_SetDirection(uint8_t direction)
{
    if(direction == 0)//正转
    {
        HAL_GPIO_WritePin(DC_DIR1_GPIO_Port, DC_DIR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DC_DIR2_GPIO_Port, DC_DIR2_Pin, GPIO_PIN_RESET);
    }
    if(direction == 1)//反转
    {
        HAL_GPIO_WritePin(DC_DIR1_GPIO_Port, DC_DIR1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DC_DIR2_GPIO_Port, DC_DIR2_Pin, GPIO_PIN_SET);
    }
    if(direction == 2)//停止
    {
        HAL_GPIO_WritePin(DC_DIR1_GPIO_Port, DC_DIR1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DC_DIR2_GPIO_Port, DC_DIR2_Pin, GPIO_PIN_RESET);
    }
}

void DC_Motor_SetSpeed(int16_t speed_rpm)
{
    uint32_t timer_period = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t pwm_max = (uint32_t)((float)(timer_period + 1U) * DC_MOTOR_MAX_DUTY_RATIO);
    float speed = (float)speed_rpm;

    if (speed < 0.0f)
    {
        speed = 0.0f;
    }
    if (speed > DC_MOTOR_MAX_SPEED_RPM)
    {
        speed = DC_MOTOR_MAX_SPEED_RPM;
    }

    /* 1800RPM 对应 98% 占空比上限 */
    uint32_t pwm_value = (uint32_t)((speed / DC_MOTOR_MAX_SPEED_RPM) * (float)(timer_period + 1U) * DC_MOTOR_MAX_DUTY_RATIO);
    if (pwm_value > pwm_max)
    {
        pwm_value = pwm_max;
    }
    if (pwm_value > timer_period)
    {
        pwm_value = timer_period;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
}