/*
 * @Author: Xiang xin wang wxinxiang8@gmail.com
 * @Date: 2026-03-18 12:57:30
 * @LastEditors: Xiang xin wang wxinxiang8@gmail.com
 * @LastEditTime: 2026-03-18 16:28:37
 * @FilePath: \MDK-ARMd:\shrimp_machine\shrimp_machine\Core\Src\IR.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "IR.h"
#include "main.h"

void IR_Init(void)
{
    //none
}

void IR_Read(void)
{
    GPIO_PinState shrimp_detected = HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin);
}
