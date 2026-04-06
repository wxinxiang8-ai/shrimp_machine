#ifndef __SCREEN_H
#define __SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------- Ring Buffer ---------- */

#define SCR_RX_DMA_SIZE    256
#define SCR_RX_RING_SIZE   512
#define SCR_FRAME_MAX      128

typedef struct {
    uint8_t  buf[SCR_RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ScreenRingBuf_t;

/* ---------- Dirty Flags ---------- */

#define DIRTY_COUNT    (1U << 0)
#define DIRTY_STATE    (1U << 1)
#define DIRTY_DC       (1U << 2)
#define DIRTY_STEPPER  (1U << 3)
#define DIRTY_PUMP     (1U << 4)
#define DIRTY_TIME     (1U << 5)
#define DIRTY_ALL      (0xFFFFFFFFU)

/* ---------- System State ---------- */

typedef enum {
    SYS_STOPPED = 0,
    SYS_RUNNING,
    SYS_FAULT
} SystemState_t;

/* ---------- Fault Flags ---------- */

#define FAULT_NONE                0x00000000U
#define FAULT_SCREEN_RX_OVERFLOW  0x00000001U
#define FAULT_SCREEN_BAD_FRAME    0x00000002U
#define FAULT_SCREEN_OFFLINE      0x00000004U

/* ---------- Machine Context ---------- */

typedef struct {
    volatile uint32_t peel_count;
    uint32_t run_time_sec;
    uint32_t run_start_tick;
    SystemState_t sys_state;
    uint32_t fault_flags;
    uint16_t dc_target_rpm;
    uint8_t  dc_dir;
    uint8_t  stepper_enable;
    uint16_t stepper_target_rpm;
    uint8_t  stepper_dir;
    uint8_t  pump_on;
    volatile uint32_t dirty_mask;
    uint32_t last_refresh_tick;
} MachineContext_t;

/* ---------- App Events ---------- */

typedef enum {
    EVT_NONE = 0,
    EVT_RUN_START,
    EVT_RUN_STOP,
    EVT_DC_SPEED_SET,
    EVT_DC_DIR_SET,
    EVT_STEPPER_ENABLE,
    EVT_STEPPER_DISABLE,
    EVT_STEPPER_SPEED_SET,
    EVT_STEPPER_DIR_SET,
    EVT_PUMP_ON,
    EVT_PUMP_OFF,
    EVT_RESET_COUNT,
    EVT_PAGE_CHANGED
} AppEventType_t;

typedef struct {
    AppEventType_t type;
    uint16_t       value;
} AppEvent_t;

#define APP_EVT_QUEUE_SIZE  16

/* ---------- Touch Button Definition ---------- */

typedef struct {
    uint16_t x, y, w, h;     /* hit area */
    uint16_t bg_color;        /* normal background */
    uint16_t fg_color;        /* text color */
    const char *label;        /* display text */
    AppEventType_t evt_type;  /* event on press */
    uint16_t evt_value;       /* event value */
} TouchBtn_t;

/* ---------- Screen Public API ---------- */

void     Screen_Init(void);
void     Screen_ProcessRx(void);
void     Screen_RefreshDirty(MachineContext_t *ctx);
void     Screen_DrawFullUI(MachineContext_t *ctx);

void     Screen_SendCmd(const char *cmd);
void     Screen_UART_RxCallback(uint16_t size);

/* ---------- App Public API ---------- */

void     App_Init(MachineContext_t *ctx);
void     App_PostEvent(AppEventType_t type, uint16_t value);
void     App_ProcessEvents(MachineContext_t *ctx);
void     App_ApplyOutputs(MachineContext_t *ctx);
void     App_UpdateRuntime(MachineContext_t *ctx);

/* ---------- IR Counter ---------- */

void     IR_Counter_Callback(MachineContext_t *ctx);

/* ---------- Externs ---------- */

extern uint8_t scr_dma_buf[SCR_RX_DMA_SIZE];
extern MachineContext_t g_ctx;

#ifdef __cplusplus
}
#endif

#endif /* __SCREEN_H */
