#include "dump.h"

void Dump_Init(void)
{
    Dump_Stop();
}

void Dump_Start(void)
{
    HAL_GPIO_WritePin(Water_Pump_GPIO_Port, Water_Pump_Pin, GPIO_PIN_SET);
}

void Dump_Stop(void)
{
    HAL_GPIO_WritePin(Water_Pump_GPIO_Port, Water_Pump_Pin, GPIO_PIN_RESET);
}

void Dump_SetState(bool enabled)
{
    if (enabled)
    {
        Dump_Start();
    }
    else
    {
        Dump_Stop();
    }
}
