#include "machine_workflow.h"

#include <string.h>
#include <stdbool.h>

#include "dc_motor.h"
#include "dump.h"
#include "emm_motor.h"
#include "Emm_V5.h"
#include "rm3508.h"

#define MACHINE_DC_DEFAULT_RPM          1200U
#define MACHINE_DC_MIN_RPM               800U
#define MACHINE_DC_MAX_RPM              1600U
#define MACHINE_DC_RAMP_STEP_RPM          50U
#define MACHINE_DC_RAMP_INTERVAL_MS      200U

#define MACHINE_RM3508_MOTOR_ID            0U
#define MACHINE_RM3508_TARGET_RPM       2000
#define MACHINE_RM3508_CW_POLARITY         1

#define MACHINE_EMM_MAIN_ID                4U
#define MACHINE_EMM_MAIN_DIR_CW            1U
#define MACHINE_EMM_MAIN_SPEED_RPM        50U
#define MACHINE_EMM_MAIN_ACCEL            10U

#define MACHINE_EMM_TRIGGER_ID             1U
#define MACHINE_EMM_TRIGGER_DIR_CW         0U
#define MACHINE_EMM_TRIGGER_SPEED_RPM     50U
#define MACHINE_EMM_TRIGGER_ACCEL         10U
#define EMM_ID1_ONE_TURN_PULSES        3200UL
#define MACHINE_EMM_CMD_GAP_MS            20U
#define MACHINE_EMM_STARTUP_DELAY_MS    2000U
#define MACHINE_EMM_ENABLE_DELAY_MS      100U

#define IR_DEBOUNCE_MS                     50U
#define SCREEN_TIME_INTERVAL_MS          1000U

typedef struct {
    AppEvent_t queue[APP_EVT_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} WorkflowEventQueue_t;

typedef struct {
    uint16_t dc_applied_rpm;
    uint16_t last_dc_command_rpm;
    uint32_t last_dc_ramp_tick;
    uint32_t last_runtime_tick;
    uint32_t last_ir_tick;
    uint8_t last_dc_dir;
    uint8_t last_pump_on;
    uint8_t emm_main_running;
    uint8_t outputs_stopped;
    uint8_t hardware_initialized;
    uint8_t emm_initialized;
} WorkflowState_t;

static WorkflowEventQueue_t s_evt_queue = {0};
static WorkflowState_t s_state = {0};
static uint32_t s_prev_display_sec = 0xFFFFFFFFU;

static uint16_t ClampDcRpm(uint16_t rpm)
{
    if (rpm < MACHINE_DC_MIN_RPM)
    {
        return MACHINE_DC_MIN_RPM;
    }
    if (rpm > MACHINE_DC_MAX_RPM)
    {
        return MACHINE_DC_MAX_RPM;
    }
    return rpm;
}

static void Machine_EventQueue_Reset(void)
{
    s_evt_queue.head = 0U;
    s_evt_queue.tail = 0U;
}

void App_PostEvent(AppEventType_t type, uint16_t value)
{
    uint8_t next = (uint8_t)((s_evt_queue.head + 1U) % APP_EVT_QUEUE_SIZE);
    if (next == s_evt_queue.tail)
    {
        return;
    }

    s_evt_queue.queue[s_evt_queue.head].type = type;
    s_evt_queue.queue[s_evt_queue.head].value = value;
    s_evt_queue.head = next;
}

static bool Machine_PopEvent(AppEvent_t *evt)
{
    if (s_evt_queue.head == s_evt_queue.tail)
    {
        return false;
    }

    *evt = s_evt_queue.queue[s_evt_queue.tail];
    s_evt_queue.tail = (uint8_t)((s_evt_queue.tail + 1U) % APP_EVT_QUEUE_SIZE);
    return true;
}

static void Machine_EnableEmmMotors(void)
{
    if (s_state.emm_initialized != 0U)
    {
        return;
    }

    HAL_Delay(MACHINE_EMM_STARTUP_DELAY_MS);
    Emm_V5_En_Control(MACHINE_EMM_MAIN_ID, true, false);
    HAL_Delay(MACHINE_EMM_ENABLE_DELAY_MS);
    Emm_V5_En_Control(MACHINE_EMM_TRIGGER_ID, true, false);
    HAL_Delay(MACHINE_EMM_ENABLE_DELAY_MS);
    Emm_V5_Stop_Now(MACHINE_EMM_MAIN_ID, false);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
    Emm_V5_Stop_Now(MACHINE_EMM_TRIGGER_ID, false);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);

    s_state.emm_initialized = 1U;
}

static void Machine_StopAllOutputs(MachineContext_t *ctx)
{
    (void)ctx;
    DC_Motor_SetSpeed(0);
    DC_Motor_SetDirection(2);
    RM3508_StopAll();
    Emm_V5_Stop_Now(MACHINE_EMM_MAIN_ID, false);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
    Emm_V5_Stop_Now(MACHINE_EMM_TRIGGER_ID, false);
    Dump_Stop();
    s_state.dc_applied_rpm = 0U;
    s_state.last_dc_command_rpm = 0U;
    s_state.last_dc_dir = 2U;
    s_state.last_pump_on = 0U;
    s_state.emm_main_running = 0U;
    s_state.outputs_stopped = 1U;
}

static int16_t Machine_Rm3508SignedTarget(const MachineContext_t *ctx)
{
    int16_t rpm = (ctx->stepper_enable != 0U) ? (int16_t)MACHINE_RM3508_TARGET_RPM : 0;
    return (int16_t)(rpm * MACHINE_RM3508_CW_POLARITY);
}

static void Machine_ApplyRunningOutputs(MachineContext_t *ctx)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_state.last_dc_ramp_tick) >= MACHINE_DC_RAMP_INTERVAL_MS)
    {
        s_state.last_dc_ramp_tick = now;

        if (s_state.dc_applied_rpm < ctx->dc_target_rpm)
        {
            uint16_t next = (uint16_t)(s_state.dc_applied_rpm + MACHINE_DC_RAMP_STEP_RPM);
            s_state.dc_applied_rpm = (next > ctx->dc_target_rpm) ? ctx->dc_target_rpm : next;
        }
        else if (s_state.dc_applied_rpm > ctx->dc_target_rpm)
        {
            uint16_t delta = (s_state.dc_applied_rpm > MACHINE_DC_RAMP_STEP_RPM) ? MACHINE_DC_RAMP_STEP_RPM : s_state.dc_applied_rpm;
            uint16_t next = (uint16_t)(s_state.dc_applied_rpm - delta);
            s_state.dc_applied_rpm = (next < ctx->dc_target_rpm) ? ctx->dc_target_rpm : next;
        }
    }

    if (s_state.outputs_stopped != 0U || (s_state.last_dc_dir != ctx->dc_dir))
    {
        if (ctx->dc_dir <= 1U)
        {
            DC_Motor_SetDirection(ctx->dc_dir);
            s_state.last_dc_dir = ctx->dc_dir;
        }
        else
        {
            DC_Motor_SetDirection(2U);
            s_state.last_dc_dir = 2U;
        }
    }

    if (s_state.outputs_stopped != 0U || (s_state.last_dc_command_rpm != s_state.dc_applied_rpm))
    {
        DC_Motor_SetSpeed((int16_t)s_state.dc_applied_rpm);
        s_state.last_dc_command_rpm = s_state.dc_applied_rpm;
    }

    RM3508_SetSpeed(MACHINE_RM3508_MOTOR_ID, Machine_Rm3508SignedTarget(ctx));
    RM3508_Control_Loop();

    if (s_state.outputs_stopped != 0U || s_state.emm_main_running == 0U)
    {
        Emm_V5_En_Control(MACHINE_EMM_MAIN_ID, true, false);
        HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
        Emm_V5_Vel_Control(MACHINE_EMM_MAIN_ID,
                           MACHINE_EMM_MAIN_DIR_CW,
                           MACHINE_EMM_MAIN_SPEED_RPM,
                           MACHINE_EMM_MAIN_ACCEL,
                           false);
        s_state.emm_main_running = 1U;
    }

    if (s_state.outputs_stopped != 0U || (s_state.last_pump_on != ctx->pump_on))
    {
        Dump_SetState(ctx->pump_on != 0U);
        s_state.last_pump_on = ctx->pump_on;
    }

    s_state.outputs_stopped = 0U;
}

static void Machine_StartRun(MachineContext_t *ctx)
{
    if (ctx->sys_state == SYS_RUNNING)
    {
        return;
    }

    ctx->sys_state = SYS_RUNNING;
    ctx->run_start_tick = HAL_GetTick();
    ctx->dc_target_rpm = MACHINE_DC_DEFAULT_RPM;
    ctx->dc_dir = 0U;
    ctx->stepper_enable = 1U;
    ctx->stepper_target_rpm = (uint16_t)MACHINE_RM3508_TARGET_RPM;
    ctx->stepper_dir = 0U;
    ctx->pump_on = 1U;
    s_prev_display_sec = 0xFFFFFFFFU;
    ctx->dirty_mask |= DIRTY_STATE | DIRTY_TIME | DIRTY_DC | DIRTY_STEPPER | DIRTY_PUMP;
}

static void Machine_StopRun(MachineContext_t *ctx)
{
    if (ctx->sys_state == SYS_RUNNING)
    {
        ctx->run_time_sec += (HAL_GetTick() - ctx->run_start_tick) / 1000U;
    }

    ctx->run_start_tick = 0U;
    ctx->sys_state = SYS_STOPPED;
    ctx->stepper_enable = 0U;
    ctx->pump_on = 0U;
    s_prev_display_sec = 0xFFFFFFFFU;
    Machine_StopAllOutputs(ctx);
    ctx->dirty_mask |= DIRTY_STATE | DIRTY_TIME | DIRTY_DC | DIRTY_STEPPER | DIRTY_PUMP;
}

static void Machine_HandleIrTrigger(MachineContext_t *ctx)
{
    if (ctx->sys_state != SYS_RUNNING)
    {
        return;
    }

    ctx->peel_count++;
    ctx->dirty_mask |= DIRTY_COUNT;

    Emm_V5_En_Control(MACHINE_EMM_TRIGGER_ID, true, false);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
    Emm_V5_Pos_Control(MACHINE_EMM_TRIGGER_ID,
                       MACHINE_EMM_TRIGGER_DIR_CW,
                       MACHINE_EMM_TRIGGER_SPEED_RPM,
                       MACHINE_EMM_TRIGGER_ACCEL,
                       EMM_ID1_ONE_TURN_PULSES,
                       false,
                       false);
}

void App_Init(MachineContext_t *ctx)
{
    Machine_Workflow_Init(ctx);
}

void Machine_Workflow_Init(MachineContext_t *ctx)
{
    uint8_t hardware_initialized = s_state.hardware_initialized;
    uint8_t emm_initialized = s_state.emm_initialized;

    memset(ctx, 0, sizeof(MachineContext_t));
    memset(&s_state, 0, sizeof(s_state));
    s_state.hardware_initialized = hardware_initialized;
    s_state.emm_initialized = emm_initialized;
    Machine_EventQueue_Reset();

    ctx->sys_state = SYS_STOPPED;
    ctx->dc_target_rpm = MACHINE_DC_DEFAULT_RPM;
    ctx->dc_dir = 0U;
    ctx->stepper_enable = 0U;
    ctx->stepper_target_rpm = (uint16_t)MACHINE_RM3508_TARGET_RPM;
    ctx->stepper_dir = 0U;
    ctx->pump_on = 0U;
    ctx->dirty_mask = DIRTY_ALL;

    s_prev_display_sec = 0xFFFFFFFFU;
    s_state.last_runtime_tick = HAL_GetTick();
    s_state.last_dc_ramp_tick = HAL_GetTick();

    if (s_state.hardware_initialized == 0U)
    {
        DC_Motor_Init();
        DC_Motor_SetDirection(2U);
        DC_Motor_SetSpeed(0);

        RM3508_Init();
        RM3508_StopAll();

        EMM_MOTOR_Init();
        Machine_EnableEmmMotors();

        Dump_Init();
        Machine_StopAllOutputs(ctx);

        s_state.hardware_initialized = 1U;
    }
    else
    {
        Machine_StopAllOutputs(ctx);
        Dump_Init();
    }

    s_state.outputs_stopped = 1U;
}

void App_ProcessEvents(MachineContext_t *ctx)
{
    AppEvent_t evt;

    while (Machine_PopEvent(&evt))
    {
        switch (evt.type)
        {
        case EVT_RUN_START:
            Machine_StartRun(ctx);
            break;

        case EVT_RUN_STOP:
            Machine_StopRun(ctx);
            break;

        case EVT_DC_SPEED_SET:
            ctx->dc_target_rpm = ClampDcRpm(evt.value);
            ctx->dirty_mask |= DIRTY_DC;
            break;

        case EVT_DC_DIR_SET:
            ctx->dc_dir = (uint8_t)((evt.value != 0U) ? 1U : 0U);
            ctx->dirty_mask |= DIRTY_DC;
            break;

        case EVT_STEPPER_ENABLE:
            ctx->stepper_enable = 1U;
            ctx->dirty_mask |= DIRTY_STEPPER;
            break;

        case EVT_STEPPER_DISABLE:
            ctx->stepper_enable = 0U;
            RM3508_Stop(MACHINE_RM3508_MOTOR_ID);
            ctx->dirty_mask |= DIRTY_STEPPER;
            break;

        case EVT_STEPPER_SPEED_SET:
            ctx->stepper_target_rpm = (uint16_t)MACHINE_RM3508_TARGET_RPM;
            ctx->dirty_mask |= DIRTY_STEPPER;
            break;

        case EVT_STEPPER_DIR_SET:
            ctx->stepper_dir = 0U;
            ctx->dirty_mask |= DIRTY_STEPPER;
            break;

        case EVT_PUMP_ON:
            ctx->pump_on = 1U;
            ctx->dirty_mask |= DIRTY_PUMP;
            break;

        case EVT_PUMP_OFF:
            ctx->pump_on = 0U;
            Dump_Stop();
            ctx->dirty_mask |= DIRTY_PUMP;
            break;

        case EVT_RESET_COUNT:
            ctx->peel_count = 0U;
            ctx->dirty_mask |= DIRTY_COUNT;
            break;

        case EVT_IR_TRIGGER:
            Machine_HandleIrTrigger(ctx);
            break;

        default:
            break;
        }
    }
}

void App_ApplyOutputs(MachineContext_t *ctx)
{
    if ((ctx->sys_state != SYS_RUNNING) || (ctx->fault_flags != FAULT_NONE))
    {
        if (s_state.outputs_stopped == 0U)
        {
            Machine_StopAllOutputs(ctx);
        }
        return;
    }

    Machine_ApplyRunningOutputs(ctx);
}

void App_UpdateRuntime(MachineContext_t *ctx)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_state.last_runtime_tick) < SCREEN_TIME_INTERVAL_MS)
    {
        return;
    }

    s_state.last_runtime_tick = now;

    if (ctx->sys_state == SYS_RUNNING)
    {
        uint32_t total = ctx->run_time_sec + (now - ctx->run_start_tick) / 1000U;
        if (total != s_prev_display_sec)
        {
            s_prev_display_sec = total;
            ctx->dirty_mask |= DIRTY_TIME;
        }
    }
}

void IR_Counter_Callback(MachineContext_t *ctx)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_state.last_ir_tick) < IR_DEBOUNCE_MS)
    {
        return;
    }

    s_state.last_ir_tick = now;
    App_PostEvent(EVT_IR_TRIGGER, 0U);
    (void)ctx;
}
