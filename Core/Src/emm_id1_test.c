#include "emm_id1_test.h"

#include "main.h"
#include "emm_motor.h"
#include "Emm_V5.h"
#include "gpio.h"

#define EMM_ID1_TEST_ID                  1U
#define EMM_ID1_TEST_DIR_CW              1U
#define EMM_ID1_TEST_SPEED_RPM         800U
#define EMM_ID1_TEST_ACCEL              60U
#define EMM_ID1_TEST_RUN_MS            300U
#define EMM_ID1_TEST_STOP_MS          1000U
#define EMM_ID1_TEST_CMD_GAP_MS         20U
#define EMM_ID1_TEST_ENABLE_DELAY_MS   100U
#define EMM_ID1_TEST_STARTUP_DELAY_MS 2000U
#define EMM_ID1_TEST_TRIGGER_DEBOUNCE_MS 200U

typedef struct
{
    uint8_t initialized;
    uint8_t ir_trigger_flag;
    uint8_t ir_wait_release;
    uint8_t motor_running;
    uint32_t last_ir_tick;
    uint32_t last_phase_tick;
} EmmId1TestState_t;

static EmmId1TestState_t s_id1_test = {0};

bool EmmId1Test_IsActive(void)
{
    return s_id1_test.initialized != 0U;
}

void EmmId1Test_Init(void)
{
    s_id1_test.initialized = 1U;
    s_id1_test.ir_trigger_flag = 0U;
    s_id1_test.ir_wait_release = 0U;
    s_id1_test.motor_running = 0U;
    s_id1_test.last_ir_tick = 0U;
    s_id1_test.last_phase_tick = 0U;

    EMM_MOTOR_Init();
    HAL_Delay(EMM_ID1_TEST_STARTUP_DELAY_MS);
    Emm_V5_Reset_Clog_Pro(EMM_ID1_TEST_ID);
    HAL_Delay(EMM_ID1_TEST_CMD_GAP_MS);
    Emm_V5_Modify_Ctrl_Mode(EMM_ID1_TEST_ID, false, 2);
    HAL_Delay(EMM_ID1_TEST_CMD_GAP_MS);
    Emm_V5_En_Control(EMM_ID1_TEST_ID, true, false);
    HAL_Delay(EMM_ID1_TEST_ENABLE_DELAY_MS);
    Emm_V5_Stop_Now(EMM_ID1_TEST_ID, false);
    HAL_Delay(EMM_ID1_TEST_CMD_GAP_MS);
    EMM_Vel_control(EMM_ID1_TEST_ID,
                    EMM_ID1_TEST_DIR_CW,
                    EMM_ID1_TEST_SPEED_RPM,
                    EMM_ID1_TEST_ACCEL,
                    false);
    s_id1_test.motor_running = 1U;
    s_id1_test.last_phase_tick = HAL_GetTick();
}

void EmmId1Test_Process(void)
{
    if (s_id1_test.initialized == 0U)
    {
        return;
    }

    HAL_Delay(100);
}

void EmmId1Test_IrCallback(void)
{
    uint32_t now;

    if (s_id1_test.initialized == 0U)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) != GPIO_PIN_RESET)
    {
        s_id1_test.ir_wait_release = 0U;
        return;
    }

    if (s_id1_test.ir_wait_release != 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_id1_test.last_ir_tick) < EMM_ID1_TEST_TRIGGER_DEBOUNCE_MS)
    {
        return;
    }

    s_id1_test.last_ir_tick = now;
    s_id1_test.ir_wait_release = 1U;
    s_id1_test.ir_trigger_flag = 1U;
}
