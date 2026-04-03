#include "emm_motor.h"
#include "usart.h"
/**
 * @description: 初始化ZDT电机
 * @param void
 * @return void
 */
void EMM_MOTOR_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart1); //清除IDLE标志
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //使能UART1 IDLE中断
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxCmd, CMD_LEN); // 开启DMA接收模式
}

void EMM_Vel_control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    Emm_V5_Vel_Control(addr, dir, vel, acc, snF);
}