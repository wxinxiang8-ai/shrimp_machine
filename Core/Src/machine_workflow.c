#include "machine_workflow.h"

#include <string.h>
#include <stdbool.h>

#include "dc_motor.h"
#include "dump.h"
#include "emm_motor.h"
#include "Emm_V5.h"
#include "rm3508.h"

#define MACHINE_DC_DEFAULT_RPM           800U
#define MACHINE_DC_MIN_RPM               800U
#define MACHINE_DC_MAX_RPM              1600U
#define MACHINE_DC_RAMP_STEP_RPM          50U
#define MACHINE_DC_RAMP_INTERVAL_MS      200U
#define MACHINE_DC_START_DELAY_MS       5000U
#define MACHINE_DC_IR_STOP_DELAY_MS        5U
#define MACHINE_DC_IR_DECEL_MS           0U
#define MACHINE_DC_IR_DECEL_INTERVAL_MS   50U

#define MACHINE_RM3508_MOTOR_ID            0U
#define MACHINE_RM3508_DEFAULT_RPM     7500U
#define MACHINE_RM3508_MIN_RPM          500U
#define MACHINE_RM3508_MAX_RPM         7500U
#define MACHINE_RM3508_CW_POLARITY       -1
#define MACHINE_RM3508_CTRL_INTERVAL_MS  10U
#define MACHINE_RM3508_RAMP_STEP_RPM    200U
#define MACHINE_RM3508_RAMP_INTERVAL_MS  50U

#define MACHINE_EMM_MAIN_ID                6U
#define MACHINE_EMM_MAIN_DIR_CW            0U
#define MACHINE_EMM_MAIN_SPEED_RPM       200U
#define MACHINE_EMM_MAIN_ACCEL            80U

#define MACHINE_EMM_AUX_ID                 1U
#define MACHINE_EMM_AUX_DIR_CW            1U
#define MACHINE_EMM_AUX_SPEED_RPM      1000U
#define MACHINE_EMM_AUX_ACCEL            80U
#define MACHINE_EMM_CMD_GAP_MS            20U
#define MACHINE_EMM_STARTUP_DELAY_MS    2000U
#define MACHINE_EMM_ENABLE_DELAY_MS      100U

#define MACHINE_RM3508_BURST_RPM        1200U
#define MACHINE_RM3508_BURST_DIR_CW        0U
#define MACHINE_RM3508_BURST_MS         5000U

#define IR_DEBOUNCE_MS                    200U
#define MACHINE_IR_COOLDOWN_MS          8000U
#define MACHINE_IR_STARTUP_IGNORE_MS     1500U
#define SCREEN_TIME_INTERVAL_MS          1000U

typedef struct {
    AppEvent_t queue[APP_EVT_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} WorkflowEventQueue_t;

typedef struct {
    uint16_t dc_applied_rpm;
    uint16_t last_dc_command_rpm;
    uint16_t dc_resume_rpm;
    uint16_t rm3508_resume_target_rpm;
    uint16_t dc_ir_decel_step_rpm;
    uint32_t dc_start_enable_tick;
    uint32_t dc_start_wait_until_tick;
    uint32_t dc_stop_delay_until_tick;
    uint32_t dc_ir_decel_next_tick;
    uint32_t last_dc_ramp_tick;
    uint32_t last_runtime_tick;
    uint32_t last_ir_tick;
    uint32_t ir_cooldown_until_tick;
    uint32_t last_rm3508_ctrl_tick;
    uint32_t last_rm3508_ramp_tick;
    uint32_t rm3508_burst_start_tick;
    uint32_t emm_seq_next_tick;
    int16_t last_rm3508_target_rpm;
    uint16_t rm3508_applied_rpm;
    uint8_t last_dc_dir;
    uint8_t dc_resume_dir;
    uint8_t dc_waiting_for_emm_ready;
    uint8_t dc_stop_delay_active;
    uint8_t dc_ir_decel_active;
    uint8_t last_pump_on;
    uint8_t emm_main_running;
    uint8_t ir_burst_active;
    uint8_t emm_aux_running;
    uint8_t rm3508_resume_enable;
    uint8_t rm3508_resume_dir;
    uint8_t emm_seq_mode;
    uint8_t emm_seq_step;
    uint8_t outputs_stopped;
    uint8_t hardware_initialized;
    uint8_t emm_initialized;
    uint8_t ir_wait_release;
} WorkflowState_t;

typedef enum {
    EMM_SEQ_NONE = 0,
    EMM_SEQ_MAIN_START
} EmmSequenceMode_t;

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

static uint16_t ClampRm3508Rpm(uint16_t rpm)
{
    if (rpm < MACHINE_RM3508_MIN_RPM)
    {
        return MACHINE_RM3508_MIN_RPM;
    }
    if (rpm > MACHINE_RM3508_MAX_RPM)
    {
        return MACHINE_RM3508_MAX_RPM;
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

    Emm_V5_Reset_Clog_Pro(MACHINE_EMM_MAIN_ID);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
    Emm_V5_Modify_Ctrl_Mode(MACHINE_EMM_MAIN_ID, false, 2);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
    Emm_V5_En_Control(MACHINE_EMM_MAIN_ID, true, false);
    HAL_Delay(MACHINE_EMM_ENABLE_DELAY_MS);
    Emm_V5_Stop_Now(MACHINE_EMM_MAIN_ID, false);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);

    Emm_V5_Reset_Clog_Pro(MACHINE_EMM_AUX_ID);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
    Emm_V5_Modify_Ctrl_Mode(MACHINE_EMM_AUX_ID, false, 2);
    HAL_Delay(MACHINE_EMM_CMD_GAP_MS);
    Emm_V5_En_Control(MACHINE_EMM_AUX_ID, true, false);
    HAL_Delay(MACHINE_EMM_ENABLE_DELAY_MS);
    Emm_V5_Stop_Now(MACHINE_EMM_AUX_ID, false);
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
    Emm_V5_Stop_Now(MACHINE_EMM_AUX_ID, false);
    Dump_Stop();
    s_state.dc_applied_rpm = 0U;
    s_state.last_dc_command_rpm = 0U;
    s_state.rm3508_applied_rpm = 0U;
    s_state.last_rm3508_target_rpm = 0;
    s_state.last_dc_dir = 2U;
    s_state.last_pump_on = 0U;
    s_state.emm_main_running = 0U;
    s_state.emm_aux_running = 0U;
    s_state.ir_burst_active = 0U;
    s_state.rm3508_resume_enable = 0U;
    s_state.rm3508_resume_target_rpm = 0U;
    s_state.rm3508_resume_dir = 0U;
    s_state.emm_seq_mode = EMM_SEQ_NONE;
    s_state.emm_seq_step = 0U;
    s_state.emm_seq_next_tick = 0U;
    s_state.rm3508_burst_start_tick = 0U;
    s_state.dc_start_enable_tick = 0U;
    s_state.dc_stop_delay_until_tick = 0U;
    s_state.dc_ir_decel_next_tick = 0U;
    s_state.ir_cooldown_until_tick = 0U;
    s_state.dc_resume_rpm = 0U;
    s_state.dc_resume_dir = 0U;
    s_state.dc_waiting_for_emm_ready = 0U;
    s_state.dc_ir_decel_step_rpm = 0U;
    s_state.dc_stop_delay_active = 0U;
    s_state.dc_ir_decel_active = 0U;
    s_state.outputs_stopped = 1U;
}

static int16_t Machine_Rm3508SignedTarget(const MachineContext_t *ctx)
{
    int16_t rpm;

    if (ctx->stepper_enable == 0U)
    {
        return 0;
    }

    rpm = (int16_t)ClampRm3508Rpm(ctx->stepper_target_rpm);
    if (ctx->stepper_dir != 0U)
    {
        rpm = (int16_t)(-rpm);
    }

    return (int16_t)(rpm * MACHINE_RM3508_CW_POLARITY);
}

static int16_t Machine_Rm3508AppliedTarget(const MachineContext_t *ctx)
{
    uint16_t desired_abs = 0U;
    uint16_t next_abs;
    uint32_t now = HAL_GetTick();

    if (ctx->stepper_enable != 0U)
    {
        desired_abs = ClampRm3508Rpm(ctx->stepper_target_rpm);
    }

    if ((now - s_state.last_rm3508_ramp_tick) >= MACHINE_RM3508_RAMP_INTERVAL_MS)
    {
        s_state.last_rm3508_ramp_tick = now;

        if (s_state.rm3508_applied_rpm < desired_abs)
        {
            next_abs = (uint16_t)(s_state.rm3508_applied_rpm + MACHINE_RM3508_RAMP_STEP_RPM);
            s_state.rm3508_applied_rpm = (next_abs > desired_abs) ? desired_abs : next_abs;
        }
        else if (s_state.rm3508_applied_rpm > desired_abs)
        {
            uint16_t delta = (s_state.rm3508_applied_rpm > MACHINE_RM3508_RAMP_STEP_RPM) ? MACHINE_RM3508_RAMP_STEP_RPM : s_state.rm3508_applied_rpm;
            next_abs = (uint16_t)(s_state.rm3508_applied_rpm - delta);
            s_state.rm3508_applied_rpm = (next_abs < desired_abs) ? desired_abs : next_abs;
        }
    }

    if (s_state.rm3508_applied_rpm == 0U)
    {
        return 0;
    }

    return (ctx->stepper_dir != 0U) ?
           (int16_t)(-(int16_t)s_state.rm3508_applied_rpm * MACHINE_RM3508_CW_POLARITY) :
           (int16_t)((int16_t)s_state.rm3508_applied_rpm * MACHINE_RM3508_CW_POLARITY);
}

static void Machine_ApplyRunningOutputs(MachineContext_t *ctx)
{
    uint32_t now = HAL_GetTick();
    uint16_t dc_target_rpm = ctx->dc_target_rpm;
    uint8_t dc_dir = ctx->dc_dir;
    int16_t rm3508_target_rpm = Machine_Rm3508AppliedTarget(ctx);

    if ((s_state.ir_wait_release != 0U) && (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) != GPIO_PIN_RESET))
    {
        s_state.ir_wait_release = 0U;
    }

    if ((s_state.emm_seq_mode != EMM_SEQ_NONE) && (now >= s_state.emm_seq_next_tick))
    {
        switch (s_state.emm_seq_mode)
        {
        case EMM_SEQ_MAIN_START:
            if (s_state.emm_seq_step == 0U)
            {
                Emm_V5_En_Control(MACHINE_EMM_MAIN_ID, true, false);
                s_state.emm_seq_step = 1U;
                s_state.emm_seq_next_tick = now + MACHINE_EMM_CMD_GAP_MS;
            }
            else if (s_state.emm_seq_step == 1U)
            {
                Emm_V5_Vel_Control(MACHINE_EMM_MAIN_ID,
                                   MACHINE_EMM_MAIN_DIR_CW,
                                   MACHINE_EMM_MAIN_SPEED_RPM,
                                   MACHINE_EMM_MAIN_ACCEL,
                                   false);
                s_state.emm_main_running = 1U;
                s_state.emm_seq_step = 2U;
                s_state.emm_seq_next_tick = now + MACHINE_EMM_CMD_GAP_MS;
            }
            else if (s_state.emm_seq_step == 2U)
            {
                Emm_V5_Reset_Clog_Pro(MACHINE_EMM_AUX_ID);
                s_state.emm_seq_step = 3U;
                s_state.emm_seq_next_tick = now + MACHINE_EMM_CMD_GAP_MS;
            }
            else if (s_state.emm_seq_step == 3U)
            {
                Emm_V5_Modify_Ctrl_Mode(MACHINE_EMM_AUX_ID, false, 2);
                s_state.emm_seq_step = 4U;
                s_state.emm_seq_next_tick = now + MACHINE_EMM_CMD_GAP_MS;
            }
            else if (s_state.emm_seq_step == 4U)
            {
                Emm_V5_En_Control(MACHINE_EMM_AUX_ID, true, false);
                s_state.emm_seq_step = 5U;
                s_state.emm_seq_next_tick = now + MACHINE_EMM_ENABLE_DELAY_MS;
            }
            else if (s_state.emm_seq_step == 5U)
            {
                Emm_V5_Stop_Now(MACHINE_EMM_AUX_ID, false);
                s_state.emm_seq_step = 6U;
                s_state.emm_seq_next_tick = now + MACHINE_EMM_CMD_GAP_MS;
            }
            else
            {
                Emm_V5_Vel_Control(MACHINE_EMM_AUX_ID,
                                   MACHINE_EMM_AUX_DIR_CW,
                                   MACHINE_EMM_AUX_SPEED_RPM,
                                   MACHINE_EMM_AUX_ACCEL,
                                   false);
                s_state.emm_aux_running = 1U;
                s_state.dc_start_enable_tick = now + MACHINE_DC_START_DELAY_MS;
                s_state.emm_seq_mode = EMM_SEQ_NONE;
                s_state.emm_seq_step = 0U;
            }
            break;

        default:
            s_state.emm_seq_mode = EMM_SEQ_NONE;
            s_state.emm_seq_step = 0U;
            break;
        }
    }

    if (s_state.dc_waiting_for_emm_ready != 0U)
    {
        if ((s_state.dc_start_enable_tick != 0U) &&
            ((int32_t)(now - s_state.dc_start_enable_tick) >= 0))
        {
            s_state.dc_waiting_for_emm_ready = 0U;
            s_state.dc_start_enable_tick = 0U;
        }

        if (s_state.dc_waiting_for_emm_ready != 0U)
        {
            dc_target_rpm = 0U;
        }
    }

    if (s_state.ir_burst_active != 0U)
    {
        dc_dir = s_state.dc_resume_dir;

        if ((s_state.dc_stop_delay_active != 0U) &&
            ((int32_t)(now - s_state.dc_stop_delay_until_tick) < 0))
        {
            dc_target_rpm = s_state.dc_resume_rpm;
        }
        else
        {
            if (s_state.dc_stop_delay_active != 0U)
            {
                uint16_t step_count = (uint16_t)(MACHINE_DC_IR_DECEL_MS / MACHINE_DC_IR_DECEL_INTERVAL_MS);
                if (step_count == 0U)
                {
                    step_count = 1U;
                }

                s_state.dc_stop_delay_active = 0U;
                s_state.dc_stop_delay_until_tick = 0U;
                s_state.dc_ir_decel_active = 1U;
                s_state.dc_ir_decel_next_tick = now;
                s_state.dc_ir_decel_step_rpm = (uint16_t)((s_state.dc_resume_rpm + step_count - 1U) / step_count);
                if (s_state.dc_ir_decel_step_rpm == 0U)
                {
                    s_state.dc_ir_decel_step_rpm = 1U;
                }
            }

            if (s_state.dc_ir_decel_active != 0U)
            {
                dc_target_rpm = s_state.dc_applied_rpm;
                if ((int32_t)(now - s_state.dc_ir_decel_next_tick) >= 0)
                {
                    if (s_state.dc_applied_rpm > s_state.dc_ir_decel_step_rpm)
                    {
                        s_state.dc_applied_rpm = (uint16_t)(s_state.dc_applied_rpm - s_state.dc_ir_decel_step_rpm);
                    }
                    else
                    {
                        s_state.dc_applied_rpm = 0U;
                    }

                    s_state.dc_ir_decel_next_tick = now + MACHINE_DC_IR_DECEL_INTERVAL_MS;
                    s_state.last_dc_command_rpm = s_state.dc_applied_rpm + 1U;
                    if (s_state.dc_applied_rpm == 0U)
                    {
                        s_state.dc_ir_decel_active = 0U;
                        s_state.dc_ir_decel_next_tick = 0U;
                        s_state.dc_ir_decel_step_rpm = 0U;
                    }
                }
            }
            else
            {
                dc_target_rpm = 0U;
            }
        }

        if ((now - s_state.rm3508_burst_start_tick) >= MACHINE_RM3508_BURST_MS)
        {
            s_state.ir_burst_active = 0U;
            s_state.dc_stop_delay_active = 0U;
            s_state.dc_stop_delay_until_tick = 0U;
            s_state.dc_ir_decel_active = 0U;
            s_state.dc_ir_decel_next_tick = 0U;
            s_state.dc_ir_decel_step_rpm = 0U;
            ctx->stepper_enable = s_state.rm3508_resume_enable;
            ctx->stepper_target_rpm = s_state.rm3508_resume_target_rpm;
            ctx->stepper_dir = s_state.rm3508_resume_dir;
            s_state.last_dc_ramp_tick = now;
            ctx->dirty_mask |= DIRTY_DC | DIRTY_STEPPER;
        }
        else
        {
            ctx->stepper_enable = 1U;
            ctx->stepper_target_rpm = MACHINE_RM3508_BURST_RPM;
            ctx->stepper_dir = MACHINE_RM3508_BURST_DIR_CW;
        }
    }

    if ((now - s_state.last_dc_ramp_tick) >= MACHINE_DC_RAMP_INTERVAL_MS)
    {
        s_state.last_dc_ramp_tick = now;

        if (s_state.dc_applied_rpm < dc_target_rpm)
        {
            uint16_t next = (uint16_t)(s_state.dc_applied_rpm + MACHINE_DC_RAMP_STEP_RPM);
            s_state.dc_applied_rpm = (next > dc_target_rpm) ? dc_target_rpm : next;
        }
        else if (s_state.dc_applied_rpm > dc_target_rpm)
        {
            uint16_t delta = (s_state.dc_applied_rpm > MACHINE_DC_RAMP_STEP_RPM) ? MACHINE_DC_RAMP_STEP_RPM : s_state.dc_applied_rpm;
            uint16_t next = (uint16_t)(s_state.dc_applied_rpm - delta);
            s_state.dc_applied_rpm = (next < dc_target_rpm) ? dc_target_rpm : next;
        }
    }

    if (s_state.outputs_stopped != 0U || (s_state.last_dc_dir != dc_dir))
    {
        if (dc_dir <= 1U)
        {
            DC_Motor_SetDirection(dc_dir);
            s_state.last_dc_dir = dc_dir;
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

    rm3508_target_rpm = Machine_Rm3508AppliedTarget(ctx);
    if (ctx->stepper_enable != 0U)
    {
        RM3508_SetSpeed(MACHINE_RM3508_MOTOR_ID, rm3508_target_rpm);
        s_state.last_rm3508_target_rpm = rm3508_target_rpm;
    }
    else if (s_state.outputs_stopped != 0U || (s_state.last_rm3508_target_rpm != 0))
    {
        RM3508_Stop(MACHINE_RM3508_MOTOR_ID);
        s_state.last_rm3508_target_rpm = 0;
    }

    if ((ctx->stepper_enable != 0U) && ((now - s_state.last_rm3508_ctrl_tick) >= MACHINE_RM3508_CTRL_INTERVAL_MS))
    {
        s_state.last_rm3508_ctrl_tick = now;
    }

    if (ctx->sys_state == SYS_RUNNING)
    {
        if ((s_state.outputs_stopped != 0U || s_state.emm_main_running == 0U || s_state.emm_aux_running == 0U) &&
            (s_state.emm_seq_mode == EMM_SEQ_NONE))
        {
            s_state.emm_seq_mode = EMM_SEQ_MAIN_START;
            s_state.emm_seq_step = 0U;
            s_state.emm_seq_next_tick = now;
        }
    }
    else if (s_state.emm_main_running != 0U || s_state.emm_aux_running != 0U)
    {
        Emm_V5_Stop_Now(MACHINE_EMM_MAIN_ID, false);
        Emm_V5_En_Control(MACHINE_EMM_MAIN_ID, false, false);
        Emm_V5_Stop_Now(MACHINE_EMM_AUX_ID, false);
        Emm_V5_En_Control(MACHINE_EMM_AUX_ID, false, false);
        s_state.emm_main_running = 0U;
        s_state.emm_aux_running = 0U;
        s_state.ir_burst_active = 0U;
        s_state.emm_seq_mode = EMM_SEQ_NONE;
        s_state.emm_seq_step = 0U;
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
    s_state.dc_start_enable_tick = 0U;
    s_state.dc_waiting_for_emm_ready = 1U;
    ctx->stepper_enable = 0U;
    ctx->stepper_target_rpm = (uint16_t)MACHINE_RM3508_DEFAULT_RPM;
    ctx->stepper_dir = 0U;
    ctx->pump_on = 1U;
    s_state.ir_burst_active = 0U;
    s_state.ir_cooldown_until_tick = 0U;
    s_state.dc_waiting_for_emm_ready = 1U;
    s_state.dc_stop_delay_until_tick = 0U;
    s_state.dc_ir_decel_next_tick = 0U;
    s_state.dc_stop_delay_active = 0U;
    s_state.dc_ir_decel_active = 0U;
    s_state.dc_ir_decel_step_rpm = 0U;
    s_state.emm_aux_running = 0U;
    s_state.emm_main_running = 0U;
    s_state.rm3508_resume_enable = 0U;
    s_state.rm3508_resume_target_rpm = 0U;
    s_state.rm3508_resume_dir = 0U;
    s_state.last_dc_ramp_tick = HAL_GetTick();
    s_state.last_rm3508_ramp_tick = HAL_GetTick();
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
    s_state.dc_start_enable_tick = 0U;
    s_state.dc_waiting_for_emm_ready = 0U;
    s_state.dc_start_wait_until_tick = 0U;
    s_state.dc_stop_delay_until_tick = 0U;
    s_state.dc_ir_decel_next_tick = 0U;
    s_state.dc_stop_delay_active = 0U;
    s_state.dc_ir_decel_active = 0U;
    s_state.dc_ir_decel_step_rpm = 0U;
    s_state.ir_cooldown_until_tick = 0U;
    ctx->stepper_enable = 0U;
    ctx->pump_on = 0U;
    s_state.ir_burst_active = 0U;
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

    if ((HAL_GetTick() - ctx->run_start_tick) < MACHINE_IR_STARTUP_IGNORE_MS)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) != GPIO_PIN_RESET)
    {
        return;
    }

    if (s_state.ir_burst_active != 0U)
    {
        return;
    }

    if ((s_state.ir_cooldown_until_tick != 0U) &&
        ((int32_t)(HAL_GetTick() - s_state.ir_cooldown_until_tick) < 0))
    {
        return;
    }

    s_state.ir_cooldown_until_tick = HAL_GetTick() + MACHINE_IR_COOLDOWN_MS;
    s_state.dc_resume_rpm = ctx->dc_target_rpm;
    s_state.dc_resume_dir = ctx->dc_dir;
    s_state.rm3508_resume_enable = ctx->stepper_enable;
    s_state.rm3508_resume_target_rpm = ctx->stepper_target_rpm;
    s_state.rm3508_resume_dir = ctx->stepper_dir;
    s_state.rm3508_burst_start_tick = HAL_GetTick();
    s_state.dc_stop_delay_until_tick = HAL_GetTick() + MACHINE_DC_IR_STOP_DELAY_MS;
    s_state.dc_ir_decel_next_tick = 0U;
    s_state.dc_ir_decel_step_rpm = 0U;
    s_state.dc_stop_delay_active = 1U;
    s_state.dc_ir_decel_active = 0U;
    s_state.ir_burst_active = 1U;
    ctx->stepper_enable = 1U;
    ctx->stepper_target_rpm = MACHINE_RM3508_BURST_RPM;
    ctx->stepper_dir = MACHINE_RM3508_BURST_DIR_CW;
    ctx->dirty_mask |= DIRTY_DC | DIRTY_STEPPER;
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
    ctx->stepper_target_rpm = (uint16_t)MACHINE_RM3508_DEFAULT_RPM;
    ctx->stepper_dir = 0U;
    ctx->pump_on = 0U;
    ctx->dirty_mask = DIRTY_ALL;

    s_prev_display_sec = 0xFFFFFFFFU;
    s_state.last_runtime_tick = HAL_GetTick();
    s_state.last_dc_ramp_tick = HAL_GetTick();
    s_state.last_rm3508_ctrl_tick = HAL_GetTick();
    s_state.last_rm3508_ramp_tick = HAL_GetTick();

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
            ctx->stepper_target_rpm = ClampRm3508Rpm(evt.value);
            ctx->dirty_mask |= DIRTY_STEPPER;
            break;

        case EVT_STEPPER_DIR_SET:
            ctx->stepper_dir = (uint8_t)((evt.value != 0U) ? 1U : 0U);
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

    if (HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) != GPIO_PIN_RESET)
    {
        s_state.ir_wait_release = 0U;
        return;
    }

    if (s_state.ir_wait_release != 0U)
    {
        return;
    }

    if ((now - s_state.last_ir_tick) < IR_DEBOUNCE_MS)
    {
        return;
    }

    s_state.last_ir_tick = now;
    s_state.ir_wait_release = 1U;
    App_PostEvent(EVT_IR_TRIGGER, 0U);
    (void)ctx;
}
