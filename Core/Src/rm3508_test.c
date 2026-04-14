/*
 * @Author: Xiang xin wang wxinxiang8@gmail.com
 * @Date: 2026-04-12 20:41:38
 * @LastEditors: Xiang xin wang wxinxiang8@gmail.com
 * @LastEditTime: 2026-04-12 20:42:16
 * @FilePath: \MDK-ARMd:\shrimp_machine\shrimp_machine\Core\Src\rm3508_test.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "rm3508_test.h"

#include "main.h"
#include "rm3508.h"

#define RM3508_TEST_MOTOR_ID            0U
#define RM3508_TEST_TARGET_RPM       1100U
#define RM3508_TEST_SERVICE_MS         10U
#define RM3508_TEST_START_DELAY_MS    300U

typedef struct
{
    uint8_t initialized;
    uint8_t target_applied;
    uint32_t last_service_tick;
    uint32_t init_tick;
} Rm3508TestState_t;

static Rm3508TestState_t s_rm3508_test = {0};

bool Rm3508Test_IsActive(void)
{
    return s_rm3508_test.initialized != 0U;
}

void Rm3508Test_Init(void)
{
    s_rm3508_test.initialized = 1U;
    s_rm3508_test.target_applied = 0U;
    s_rm3508_test.last_service_tick = HAL_GetTick();
    s_rm3508_test.init_tick = HAL_GetTick();

    RM3508_Init();
    RM3508_StopAll();
}

void Rm3508Test_Process(void)
{
    uint32_t now;

    if (s_rm3508_test.initialized == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    if ((s_rm3508_test.target_applied == 0U) &&
        ((now - s_rm3508_test.init_tick) >= RM3508_TEST_START_DELAY_MS))
    {
        RM3508_SetSpeed(RM3508_TEST_MOTOR_ID, (int16_t)RM3508_TEST_TARGET_RPM);
        s_rm3508_test.target_applied = 1U;
    }

    if ((now - s_rm3508_test.last_service_tick) >= RM3508_TEST_SERVICE_MS)
    {
        s_rm3508_test.last_service_tick = now;
        RM3508_Service();
    }
}
