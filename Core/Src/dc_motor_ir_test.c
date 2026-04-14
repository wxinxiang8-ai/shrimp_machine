#include "dc_motor_ir_test.h"

#include "main.h"
#include "dc_motor.h"
#include "gpio.h"

#define DC_MOTOR_IR_TEST_SPEED_RPM            800U
#define DC_MOTOR_IR_TEST_STOP_MS             2000U
#define DC_MOTOR_IR_TEST_COOLDOWN_MS         5000U
#define DC_MOTOR_IR_TEST_DEBOUNCE_MS          200U
#define DC_MOTOR_IR_TEST_DIR_FORWARD            0U
#define DC_MOTOR_IR_TEST_DIR_STOP               2U

typedef struct
{
    uint8_t initialized;
    uint8_t motor_stopped;
    uint8_t ir_wait_release;
    uint32_t last_ir_tick;
    uint32_t stop_until_tick;
    uint32_t cooldown_until_tick;
} DcMotorIrTestState_t;

static DcMotorIrTestState_t s_dc_motor_ir_test = {0};

bool DcMotorIrTest_IsActive(void)
{
    return s_dc_motor_ir_test.initialized != 0U;
}

void DcMotorIrTest_Init(void)
{
    s_dc_motor_ir_test.initialized = 1U;
    s_dc_motor_ir_test.motor_stopped = 0U;
    s_dc_motor_ir_test.ir_wait_release = 0U;
    s_dc_motor_ir_test.last_ir_tick = 0U;
    s_dc_motor_ir_test.stop_until_tick = 0U;
    s_dc_motor_ir_test.cooldown_until_tick = 0U;

    DC_Motor_Init();
    DC_Motor_SetDirection(DC_MOTOR_IR_TEST_DIR_FORWARD);
    DC_Motor_SetSpeed((int16_t)DC_MOTOR_IR_TEST_SPEED_RPM);
}

void DcMotorIrTest_Process(void)
{
    uint32_t now;

    if (s_dc_motor_ir_test.initialized == 0U)
    {
        return;
    }

    if (s_dc_motor_ir_test.motor_stopped == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((int32_t)(now - s_dc_motor_ir_test.stop_until_tick) < 0)
    {
        return;
    }

    DC_Motor_SetDirection(DC_MOTOR_IR_TEST_DIR_FORWARD);
    DC_Motor_SetSpeed((int16_t)DC_MOTOR_IR_TEST_SPEED_RPM);
    s_dc_motor_ir_test.motor_stopped = 0U;
    s_dc_motor_ir_test.stop_until_tick = 0U;
}

void DcMotorIrTest_IrCallback(void)
{
    uint32_t now;

    if (s_dc_motor_ir_test.initialized == 0U)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) != GPIO_PIN_RESET)
    {
        s_dc_motor_ir_test.ir_wait_release = 0U;
        return;
    }

    if (s_dc_motor_ir_test.ir_wait_release != 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_dc_motor_ir_test.last_ir_tick) < DC_MOTOR_IR_TEST_DEBOUNCE_MS)
    {
        return;
    }

    s_dc_motor_ir_test.last_ir_tick = now;
    s_dc_motor_ir_test.ir_wait_release = 1U;

    if ((s_dc_motor_ir_test.cooldown_until_tick != 0U) &&
        ((int32_t)(now - s_dc_motor_ir_test.cooldown_until_tick) < 0))
    {
        return;
    }

    DC_Motor_SetSpeed(0);
    DC_Motor_SetDirection(DC_MOTOR_IR_TEST_DIR_STOP);
    s_dc_motor_ir_test.motor_stopped = 1U;
    s_dc_motor_ir_test.stop_until_tick = now + DC_MOTOR_IR_TEST_STOP_MS;
    s_dc_motor_ir_test.cooldown_until_tick = now + DC_MOTOR_IR_TEST_COOLDOWN_MS;
}
