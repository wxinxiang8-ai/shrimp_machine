/**
  * Screen.c - TJC Serial Screen Pure-Code GUI Module
  *
  * All UI is drawn by MCU via TJC GUI commands (xstr/fill/draw).
  * Touch input via sendxy=1 coordinate events.
  *
  * Minimal HMI project required:
  *   1. New project -> TJC8048X270_011C, 800x480, baud 115200
  *   2. Add 1 blank page (page0), black background
  *   3. page0 Pre-init event code:
  *        sendxy=1
  *        thup=1
  *   4. Compile -> .tft -> SD card -> flash
  */

#include "Screen.h"
#include "usart.h"
#include "dc_motor.h"
#include "Emm_V5.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Forward declarations */
static void Screen_HandleButtonPress(uint8_t idx, MachineContext_t *ctx);
static void Screen_ParseFrame(const uint8_t *buf, uint16_t len, MachineContext_t *ctx);

/* ================================================================
 *  TJC Color Constants (RGB565)
 * ================================================================ */

#define COL_BLACK    0
#define COL_WHITE    65535
#define COL_RED      63488
#define COL_GREEN    2016
#define COL_BLUE     31
#define COL_YELLOW   65504
#define COL_GRAY     33840
#define COL_DGRAY    16904
#define COL_ORANGE   64512
#define COL_CYAN     2047

/* ================================================================
 *  Layout Constants (800x480)
 * ================================================================ */

/* Top title bar */
#define BAR_Y       0
#define BAR_H       48

/* Left panel: peel count */
#define CNT_X       20
#define CNT_Y       60
#define CNT_NUM_Y   100

/* Right panel: state + time */
#define STA_X       420
#define STA_Y       60

/* Button row */
#define BTN_Y       200
#define BTN_H       60
#define BTN_W       160
#define BTN_GAP     20

/* Bottom: motor / pump controls */
#define BOT_Y       290
#define BOT_H       170

/* ================================================================
 *  Touch Button Definitions
 * ================================================================ */

/* Button layout: [START] [STOP] [RESET] */
static const TouchBtn_t s_buttons[] = {
    /* x,    y,      w,      h,      bg,        fg,        label,         event,              value */
    {  30,   BTN_Y,  BTN_W,  BTN_H,  COL_GREEN, COL_WHITE, "START",       EVT_RUN_START,      0 },
    {  210,  BTN_Y,  BTN_W,  BTN_H,  COL_RED,   COL_WHITE, "STOP",        EVT_RUN_STOP,       0 },
    {  390,  BTN_Y,  120,    BTN_H,  COL_GRAY,  COL_WHITE, "RESET",       EVT_RESET_COUNT,    0 },

    /* DC motor: [-] [+] */
    {  30,   310,    80,     55,     COL_DGRAY, COL_WHITE, "DC-",         EVT_DC_SPEED_SET,   0 },  /* special: decrease */
    {  280,  310,    80,     55,     COL_DGRAY, COL_WHITE, "DC+",         EVT_DC_SPEED_SET,   1 },  /* special: increase */

    /* DC direction toggle */
    {  30,   380,    160,    50,     COL_ORANGE,COL_WHITE, "DIR:FWD",     EVT_DC_DIR_SET,     0 },

    /* Stepper: [ON/OFF] [-] [+] */
    {  420,  310,    130,    55,     COL_GRAY,  COL_WHITE, "STEP:OFF",    EVT_STEPPER_ENABLE, 0 },
    {  560,  310,    70,     55,     COL_DGRAY, COL_WHITE, "S-",          EVT_STEPPER_SPEED_SET, 0 },
    {  640,  310,    70,     55,     COL_DGRAY, COL_WHITE, "S+",          EVT_STEPPER_SPEED_SET, 1 },

    /* Water pump */
    {  420,  380,    180,    55,     COL_BLUE,  COL_WHITE, "PUMP:OFF",    EVT_PUMP_ON,        0 },
};

#define BTN_COUNT  (sizeof(s_buttons) / sizeof(s_buttons[0]))

/* Mutable button labels (for toggle states) */
#define BTN_IDX_DC_DIR     5
#define BTN_IDX_STEP_ENA   6
#define BTN_IDX_PUMP       9

/* DC speed step */
#define DC_SPEED_STEP      100
#define STEP_SPEED_STEP    50

/* ================================================================
 *  Private Variables
 * ================================================================ */

static ScreenRingBuf_t s_ring = {0};
static uint8_t  s_frame_buf[SCR_FRAME_MAX];
static uint16_t s_frame_len = 0;
static uint8_t  s_ff_cnt    = 0;
static uint8_t  s_rx_byte;

uint8_t scr_dma_buf[SCR_RX_DMA_SIZE];

static const uint8_t TJC_END[3] = {0xFF, 0xFF, 0xFF};
static char s_tx_buf[256];

#define SCREEN_REFRESH_INTERVAL_MS  200
#define SCREEN_TIME_INTERVAL_MS     1000

/* Output shadow */
typedef struct {
    uint8_t  safe_stopped;
    uint16_t dc_target_rpm;
    uint8_t  dc_dir;
    uint8_t  stepper_enable;
    uint16_t stepper_target_rpm;
    uint8_t  stepper_dir;
    uint8_t  pump_on;
} OutputShadow_t;

static OutputShadow_t s_last_out = { .safe_stopped = 1U };
static uint32_t s_prev_display_sec = 0xFFFFFFFFU;

/* ================================================================
 *  Atomic Dirty Mask
 * ================================================================ */

static uint32_t DirtyMask_Take(volatile uint32_t *mask)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t snapshot = *mask;
    *mask = 0U;
    if (primask == 0U) __enable_irq();
    return snapshot;
}

/* ================================================================
 *  Ring Buffer
 * ================================================================ */

static void RingBuf_Write(ScreenRingBuf_t *rb, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next = (rb->head + 1) % SCR_RX_RING_SIZE;
        if (next == rb->tail) return;
        rb->buf[rb->head] = data[i];
        rb->head = next;
    }
}

static bool RingBuf_ReadByte(ScreenRingBuf_t *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) return false;
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % SCR_RX_RING_SIZE;
    return true;
}

/* ================================================================
 *  TJC Transmit Helpers
 * ================================================================ */

static void Screen_Transmit(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
}

static bool Screen_Fmt(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(s_tx_buf, sizeof(s_tx_buf), fmt, args);
    va_end(args);
    if (len <= 0 || len >= (int)sizeof(s_tx_buf)) return false;
    Screen_Transmit((const uint8_t *)s_tx_buf, (uint16_t)len);
    Screen_Transmit(TJC_END, 3);
    return true;
}

void Screen_SendCmd(const char *cmd)
{
    Screen_Transmit((const uint8_t *)cmd, strlen(cmd));
    Screen_Transmit(TJC_END, 3);
}

/* ================================================================
 *  TJC GUI Drawing Primitives
 * ================================================================ */

/* Fill rectangle */
static void GUI_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    Screen_Fmt("fill %u,%u,%u,%u,%u", x, y, w, h, color);
}

/* Draw hollow rectangle */
static void GUI_DrawRect(uint16_t x, uint16_t y, uint16_t x2, uint16_t y2, uint16_t color)
{
    Screen_Fmt("draw %u,%u,%u,%u,%u", x, y, x2, y2, color);
}

/* Draw text (xstr command)
 * fontid: font resource index (0 = first imported font)
 * xcen: 0=left, 1=center, 2=right
 * ycen: 0=top, 1=center, 2=bottom
 * sta: 0=crop, 1=solid bg, 2=transparent bg  */
static void GUI_Text(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                     uint8_t fontid, uint16_t pco, uint16_t bco,
                     uint8_t xcen, uint8_t ycen, uint8_t sta,
                     const char *txt)
{
    Screen_Fmt("xstr %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,\"%s\"",
               x, y, w, h, fontid, pco, bco, xcen, ycen, sta, txt);
}

/* Shorthand: centered text with solid bg */
static void GUI_Label(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      uint16_t fg, uint16_t bg, const char *txt)
{
    GUI_Text(x, y, w, h, 0, fg, bg, 1, 1, 1, txt);
}

/* Shorthand: transparent centered text */
static void GUI_TextTr(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       uint16_t fg, const char *txt)
{
    GUI_Text(x, y, w, h, 0, fg, 0, 1, 1, 2, txt);
}

/* Draw horizontal line */
static void GUI_HLine(uint16_t x1, uint16_t y, uint16_t x2, uint16_t color)
{
    Screen_Fmt("line %u,%u,%u,%u,%u", x1, y, x2, y, color);
}

/* Draw filled progress bar */
static void GUI_ProgressBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                            uint16_t bg, uint16_t fg, uint32_t pct)
{
    if (pct > 100) pct = 100;
    GUI_FillRect(x, y, w, h, bg);
    uint16_t fill_w = (uint16_t)((uint32_t)w * pct / 100);
    if (fill_w > 0) GUI_FillRect(x, y, fill_w, h, fg);
}

/* ================================================================
 *  Draw a touch button
 * ================================================================ */

static void GUI_DrawButton(const TouchBtn_t *btn)
{
    GUI_FillRect(btn->x, btn->y, btn->w, btn->h, btn->bg_color);
    GUI_DrawRect(btn->x, btn->y, btn->x + btn->w - 1, btn->y + btn->h - 1, COL_WHITE);
    GUI_Label(btn->x, btn->y, btn->w, btn->h, btn->fg_color, btn->bg_color, btn->label);
}

/* Redraw a single button with a new label & color */
static void GUI_RedrawBtn(uint8_t idx, const char *label, uint16_t bg)
{
    if (idx >= BTN_COUNT) return;
    const TouchBtn_t *btn = &s_buttons[idx];
    GUI_FillRect(btn->x, btn->y, btn->w, btn->h, bg);
    GUI_DrawRect(btn->x, btn->y, btn->x + btn->w - 1, btn->y + btn->h - 1, COL_WHITE);
    GUI_Label(btn->x, btn->y, btn->w, btn->h, btn->fg_color, bg, label);
}

/* ================================================================
 *  Draw Full UI
 * ================================================================ */

void Screen_DrawFullUI(MachineContext_t *ctx)
{
    /* Clear screen */
    Screen_Fmt("cls %u", COL_BLACK);
    HAL_Delay(50);

    /* === Title bar === */
    GUI_FillRect(0, 0, 800, BAR_H, COL_DGRAY);
    GUI_Label(0, 0, 800, BAR_H, COL_CYAN, COL_DGRAY, "Smart Shrimp Peeler");

    /* === Separator === */
    GUI_HLine(0, BAR_H, 800, COL_GRAY);

    /* === Left: Peel Count === */
    GUI_TextTr(CNT_X, CNT_Y, 200, 30, COL_GRAY, "Peel Count");

    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)ctx->peel_count);
    GUI_Label(CNT_X, CNT_NUM_Y, 300, 70, COL_YELLOW, COL_BLACK, buf);

    /* === Right: State + Time === */
    GUI_TextTr(STA_X, STA_Y, 100, 30, COL_GRAY, "Status");
    GUI_Label(STA_X + 110, STA_Y, 200, 30, COL_RED, COL_BLACK, "STOPPED");

    GUI_TextTr(STA_X, STA_Y + 50, 100, 30, COL_GRAY, "Runtime");
    GUI_Label(STA_X + 110, STA_Y + 50, 200, 30, COL_WHITE, COL_BLACK, "00:00:00");

    /* === Separator === */
    GUI_HLine(0, BTN_Y - 10, 800, COL_GRAY);

    /* === Buttons === */
    for (uint8_t i = 0; i < BTN_COUNT; i++)
    {
        GUI_DrawButton(&s_buttons[i]);
    }

    /* === Bottom labels === */
    GUI_HLine(0, BOT_Y - 10, 800, COL_GRAY);

    /* DC motor speed display area */
    GUI_TextTr(120, 315, 150, 20, COL_GRAY, "DC Motor");
    snprintf(buf, sizeof(buf), "%u RPM", ctx->dc_target_rpm);
    GUI_Label(120, 335, 150, 25, COL_WHITE, COL_BLACK, buf);

    /* DC progress bar */
    GUI_ProgressBar(30, 370, 330, 6, COL_DGRAY, COL_GREEN, 0);

    /* Stepper speed display */
    GUI_TextTr(420, 385, 150, 20, COL_GRAY, "Stepper");
    snprintf(buf, sizeof(buf), "%u RPM", ctx->stepper_target_rpm);
    GUI_Label(420, 405, 150, 25, COL_WHITE, COL_BLACK, buf);

    /* Bottom separator */
    GUI_HLine(400, BOT_Y - 10, 480, COL_GRAY);
}

/* ================================================================
 *  Receive Callback
 * ================================================================ */

void Screen_UART_RxCallback(uint16_t size)
{
    (void)size;
    RingBuf_Write(&s_ring, &s_rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);
}

/* ================================================================
 *  Touch Coordinate Parsing
 *
 *  sendxy=1 returns: 0x67 XH XL YH YL EVT 0xFF 0xFF 0xFF
 *  EVT: 0x01=press, 0x00=release
 * ================================================================ */

static void Screen_HandleTouch(const uint8_t *buf, uint16_t len, MachineContext_t *ctx)
{
    if (len != 6 || buf[0] != 0x67) return;

    uint16_t tx = ((uint16_t)buf[1] << 8) | buf[2];
    uint16_t ty = ((uint16_t)buf[3] << 8) | buf[4];
    uint8_t  evt = buf[5]; /* 0x01=press, 0x00=release */

    if (evt != 0x01) return; /* only handle press */

    /* Hit test all buttons */
    for (uint8_t i = 0; i < BTN_COUNT; i++)
    {
        const TouchBtn_t *btn = &s_buttons[i];
        if (tx >= btn->x && tx < (btn->x + btn->w) &&
            ty >= btn->y && ty < (btn->y + btn->h))
        {
            Screen_HandleButtonPress(i, ctx);
            return;
        }
    }
}

static void Screen_HandleButtonPress(uint8_t idx, MachineContext_t *ctx)
{
    switch (idx)
    {
    case 0: /* START */
        App_PostEvent(EVT_RUN_START, 0);
        break;

    case 1: /* STOP */
        App_PostEvent(EVT_RUN_STOP, 0);
        break;

    case 2: /* RESET count */
        App_PostEvent(EVT_RESET_COUNT, 0);
        break;

    case 3: /* DC speed - */
    {
        uint16_t spd = ctx->dc_target_rpm;
        spd = (spd >= DC_SPEED_STEP) ? (spd - DC_SPEED_STEP) : 0;
        App_PostEvent(EVT_DC_SPEED_SET, spd);
        break;
    }
    case 4: /* DC speed + */
    {
        uint16_t spd = ctx->dc_target_rpm + DC_SPEED_STEP;
        if (spd > 1800) spd = 1800;
        App_PostEvent(EVT_DC_SPEED_SET, spd);
        break;
    }
    case 5: /* DC direction toggle */
    {
        uint8_t new_dir = ctx->dc_dir ? 0 : 1;
        App_PostEvent(EVT_DC_DIR_SET, new_dir);
        break;
    }
    case 6: /* Stepper enable toggle */
        if (ctx->stepper_enable)
            App_PostEvent(EVT_STEPPER_DISABLE, 0);
        else
            App_PostEvent(EVT_STEPPER_ENABLE, 0);
        break;

    case 7: /* Stepper speed - */
    {
        uint16_t spd = ctx->stepper_target_rpm;
        spd = (spd >= STEP_SPEED_STEP) ? (spd - STEP_SPEED_STEP) : 0;
        App_PostEvent(EVT_STEPPER_SPEED_SET, spd);
        break;
    }
    case 8: /* Stepper speed + */
    {
        uint16_t spd = ctx->stepper_target_rpm + STEP_SPEED_STEP;
        if (spd > 5000) spd = 5000;
        App_PostEvent(EVT_STEPPER_SPEED_SET, spd);
        break;
    }
    case 9: /* Pump toggle */
        if (ctx->pump_on)
            App_PostEvent(EVT_PUMP_OFF, 0);
        else
            App_PostEvent(EVT_PUMP_ON, 0);
        break;

    default:
        break;
    }
}

/* ================================================================
 *  Frame Parsing
 * ================================================================ */

static void Screen_ParseFrame(const uint8_t *buf, uint16_t len, MachineContext_t *ctx)
{
    if (len == 0) return;

    /* Touch coordinate event (0x67 header) */
    if (buf[0] == 0x67)
    {
        Screen_HandleTouch(buf, len, ctx);
        return;
    }

    /* Legacy ASCII command support (for serial-assistant debugging) */
    uint16_t v = 0;

    if (len == 5 && memcmp(buf, "RUN,1", 5) == 0)       App_PostEvent(EVT_RUN_START, 0);
    else if (len == 5 && memcmp(buf, "RUN,0", 5) == 0)   App_PostEvent(EVT_RUN_STOP, 0);
    else if (len == 6 && memcmp(buf, "RSTCNT", 6) == 0)  App_PostEvent(EVT_RESET_COUNT, 0);
    else if (len == 6 && memcmp(buf, "PUMP,1", 6) == 0)  App_PostEvent(EVT_PUMP_ON, 0);
    else if (len == 6 && memcmp(buf, "PUMP,0", 6) == 0)  App_PostEvent(EVT_PUMP_OFF, 0);
    /* Numeric commands like DCSPD,600 still supported for debug */
    else if (len > 6 && memcmp(buf, "DCSPD,", 6) == 0)
    {
        v = 0;
        for (uint16_t i = 6; i < len && buf[i] >= '0' && buf[i] <= '9'; i++)
            v = v * 10 + (buf[i] - '0');
        if (v > 1800) v = 1800;
        App_PostEvent(EVT_DC_SPEED_SET, v);
    }
}

/* ================================================================
 *  Process Rx (main loop)
 * ================================================================ */

void Screen_ProcessRx(void)
{
    uint8_t ch;

    while (RingBuf_ReadByte(&s_ring, &ch))
    {
        s_frame_buf[s_frame_len++] = ch;

        if (ch == 0xFF) s_ff_cnt++;
        else            s_ff_cnt = 0;

        if (s_ff_cnt >= 3)
        {
            uint16_t payload_len = s_frame_len - 3;
            if (payload_len > 0)
            {
                Screen_ParseFrame(s_frame_buf, payload_len, &g_ctx);
            }
            s_frame_len = 0;
            s_ff_cnt    = 0;
        }

        if (s_frame_len >= SCR_FRAME_MAX)
        {
            s_frame_len = 0;
            s_ff_cnt    = 0;
        }
    }
}

/* ================================================================
 *  Init
 * ================================================================ */

void Screen_Init(void)
{
    memset(&s_ring, 0, sizeof(s_ring));
    s_frame_len = 0;
    s_ff_cnt    = 0;
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);

    /* Ensure sendxy and thup are enabled */
    HAL_Delay(100);
    Screen_SendCmd("sendxy=1");
    Screen_SendCmd("thup=1");
}

/* ================================================================
 *  UI Refresh (redraw only changed regions)
 * ================================================================ */

void Screen_RefreshDirty(MachineContext_t *ctx)
{
    uint32_t now = HAL_GetTick();
    if ((now - ctx->last_refresh_tick) < SCREEN_REFRESH_INTERVAL_MS)
        return;
    ctx->last_refresh_tick = now;

    uint32_t mask = DirtyMask_Take(&ctx->dirty_mask);
    if (mask == 0) return;

    char buf[32];

    /* --- Peel Count --- */
    if (mask & DIRTY_COUNT)
    {
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)ctx->peel_count);
        GUI_Label(CNT_X, CNT_NUM_Y, 300, 70, COL_YELLOW, COL_BLACK, buf);
    }

    /* --- System State --- */
    if (mask & DIRTY_STATE)
    {
        const char *str;
        uint16_t color;
        switch (ctx->sys_state)
        {
        case SYS_RUNNING: str = "RUNNING"; color = COL_GREEN;  break;
        case SYS_FAULT:   str = "FAULT";   color = COL_YELLOW; break;
        default:          str = "STOPPED"; color = COL_RED;    break;
        }
        GUI_Label(STA_X + 110, STA_Y, 200, 30, color, COL_BLACK, str);
    }

    /* --- Runtime --- */
    if (mask & DIRTY_TIME)
    {
        uint32_t sec = ctx->run_time_sec;
        if (ctx->sys_state == SYS_RUNNING)
            sec += (HAL_GetTick() - ctx->run_start_tick) / 1000U;
        uint32_t h = sec / 3600;
        uint32_t m = (sec % 3600) / 60;
        uint32_t s = sec % 60;
        snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
                 (unsigned long)h, (unsigned long)m, (unsigned long)s);
        GUI_Label(STA_X + 110, STA_Y + 50, 200, 30, COL_WHITE, COL_BLACK, buf);
    }

    /* --- DC Motor --- */
    if (mask & DIRTY_DC)
    {
        snprintf(buf, sizeof(buf), "%u RPM", ctx->dc_target_rpm);
        GUI_Label(120, 335, 150, 25, COL_WHITE, COL_BLACK, buf);

        uint32_t pct = ((uint32_t)ctx->dc_target_rpm * 100) / 1800;
        GUI_ProgressBar(30, 370, 330, 6, COL_DGRAY, COL_GREEN, pct);

        /* Update direction button label */
        GUI_RedrawBtn(BTN_IDX_DC_DIR,
                      ctx->dc_dir ? "DIR:REV" : "DIR:FWD",
                      ctx->dc_dir ? COL_RED : COL_ORANGE);
    }

    /* --- Stepper --- */
    if (mask & DIRTY_STEPPER)
    {
        snprintf(buf, sizeof(buf), "%u RPM", ctx->stepper_target_rpm);
        GUI_Label(420, 405, 150, 25, COL_WHITE, COL_BLACK, buf);

        GUI_RedrawBtn(BTN_IDX_STEP_ENA,
                      ctx->stepper_enable ? "STEP:ON" : "STEP:OFF",
                      ctx->stepper_enable ? COL_GREEN : COL_GRAY);
    }

    /* --- Pump --- */
    if (mask & DIRTY_PUMP)
    {
        GUI_RedrawBtn(BTN_IDX_PUMP,
                      ctx->pump_on ? "PUMP:ON" : "PUMP:OFF",
                      ctx->pump_on ? COL_CYAN : COL_BLUE);
    }
}

/* ================================================================
 *  App Event Queue
 * ================================================================ */

static AppEvent_t s_evt_queue[APP_EVT_QUEUE_SIZE];
static volatile uint8_t s_evt_head = 0;
static volatile uint8_t s_evt_tail = 0;

void App_PostEvent(AppEventType_t type, uint16_t value)
{
    uint8_t next = (s_evt_head + 1) % APP_EVT_QUEUE_SIZE;
    if (next == s_evt_tail) return;
    s_evt_queue[s_evt_head].type  = type;
    s_evt_queue[s_evt_head].value = value;
    s_evt_head = next;
}

static bool App_PopEvent(AppEvent_t *evt)
{
    if (s_evt_head == s_evt_tail) return false;
    *evt = s_evt_queue[s_evt_tail];
    s_evt_tail = (s_evt_tail + 1) % APP_EVT_QUEUE_SIZE;
    return true;
}

/* ================================================================
 *  App Init
 * ================================================================ */

void App_Init(MachineContext_t *ctx)
{
    memset(ctx, 0, sizeof(MachineContext_t));
    memset(&s_last_out, 0, sizeof(s_last_out));
    s_last_out.safe_stopped = 1U;
    s_prev_display_sec = 0xFFFFFFFFU;
    ctx->sys_state  = SYS_STOPPED;
    ctx->dirty_mask = DIRTY_ALL;
}

/* ================================================================
 *  App Event Processing
 * ================================================================ */

void App_ProcessEvents(MachineContext_t *ctx)
{
    AppEvent_t evt;

    while (App_PopEvent(&evt))
    {
        switch (evt.type)
        {
        case EVT_RUN_START:
            if (ctx->sys_state != SYS_RUNNING)
            {
                ctx->sys_state      = SYS_RUNNING;
                ctx->run_start_tick = HAL_GetTick();
                ctx->dirty_mask    |= DIRTY_STATE | DIRTY_TIME;
                s_prev_display_sec  = 0xFFFFFFFFU;
            }
            break;

        case EVT_RUN_STOP:
            if (ctx->sys_state == SYS_RUNNING)
                ctx->run_time_sec += (HAL_GetTick() - ctx->run_start_tick) / 1000U;
            ctx->run_start_tick = 0U;
            s_prev_display_sec  = 0xFFFFFFFFU;
            ctx->sys_state      = SYS_STOPPED;
            ctx->dirty_mask    |= DIRTY_STATE | DIRTY_TIME;
            break;

        case EVT_DC_SPEED_SET:
            ctx->dc_target_rpm = evt.value;
            ctx->dirty_mask   |= DIRTY_DC;
            break;

        case EVT_DC_DIR_SET:
            ctx->dc_dir      = (uint8_t)evt.value;
            ctx->dirty_mask |= DIRTY_DC;
            break;

        case EVT_STEPPER_ENABLE:
            ctx->stepper_enable = 1;
            ctx->dirty_mask    |= DIRTY_STEPPER;
            break;

        case EVT_STEPPER_DISABLE:
            ctx->stepper_enable = 0;
            ctx->dirty_mask    |= DIRTY_STEPPER;
            break;

        case EVT_STEPPER_SPEED_SET:
            ctx->stepper_target_rpm = evt.value;
            ctx->dirty_mask        |= DIRTY_STEPPER;
            break;

        case EVT_STEPPER_DIR_SET:
            ctx->stepper_dir = (uint8_t)evt.value;
            ctx->dirty_mask |= DIRTY_STEPPER;
            break;

        case EVT_PUMP_ON:
            ctx->pump_on     = 1;
            ctx->dirty_mask |= DIRTY_PUMP;
            break;

        case EVT_PUMP_OFF:
            ctx->pump_on     = 0;
            ctx->dirty_mask |= DIRTY_PUMP;
            break;

        case EVT_RESET_COUNT:
            ctx->peel_count  = 0;
            ctx->dirty_mask |= DIRTY_COUNT;
            break;

        default:
            break;
        }
    }
}

/* ================================================================
 *  Safe Stop
 * ================================================================ */

static void App_SafeStopOutputs(void)
{
    DC_Motor_SetSpeed(0);
    DC_Motor_SetDirection(2);
    Emm_V5_Stop_Now(1, false);
    HAL_GPIO_WritePin(Water_Pump_GPIO_Port, Water_Pump_Pin, GPIO_PIN_RESET);
}

/* ================================================================
 *  Apply Outputs (shadow-based)
 * ================================================================ */

void App_ApplyOutputs(MachineContext_t *ctx)
{
    if (ctx->sys_state != SYS_RUNNING || ctx->fault_flags != FAULT_NONE)
    {
        if (!s_last_out.safe_stopped)
        {
            App_SafeStopOutputs();
            memset(&s_last_out, 0, sizeof(s_last_out));
            s_last_out.safe_stopped = 1U;
        }
        return;
    }

    if (s_last_out.safe_stopped ||
        s_last_out.dc_dir != ctx->dc_dir ||
        s_last_out.dc_target_rpm != ctx->dc_target_rpm)
    {
        DC_Motor_SetDirection(ctx->dc_dir);
        DC_Motor_SetSpeed((int16_t)ctx->dc_target_rpm);
    }

    if (s_last_out.safe_stopped ||
        s_last_out.stepper_enable != ctx->stepper_enable ||
        s_last_out.stepper_dir != ctx->stepper_dir ||
        s_last_out.stepper_target_rpm != ctx->stepper_target_rpm)
    {
        if (ctx->stepper_enable)
            Emm_V5_Vel_Control(1, ctx->stepper_dir, ctx->stepper_target_rpm, 20, false);
        else
            Emm_V5_Stop_Now(1, false);
    }

    if (s_last_out.safe_stopped || s_last_out.pump_on != ctx->pump_on)
    {
        HAL_GPIO_WritePin(Water_Pump_GPIO_Port, Water_Pump_Pin,
                          ctx->pump_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    s_last_out.safe_stopped       = 0U;
    s_last_out.dc_target_rpm      = ctx->dc_target_rpm;
    s_last_out.dc_dir             = ctx->dc_dir;
    s_last_out.stepper_enable     = ctx->stepper_enable;
    s_last_out.stepper_target_rpm = ctx->stepper_target_rpm;
    s_last_out.stepper_dir        = ctx->stepper_dir;
    s_last_out.pump_on            = ctx->pump_on;
}

/* ================================================================
 *  Runtime Tracking
 * ================================================================ */

static uint32_t s_last_time_tick = 0;

void App_UpdateRuntime(MachineContext_t *ctx)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_time_tick) < SCREEN_TIME_INTERVAL_MS) return;
    s_last_time_tick = now;

    if (ctx->sys_state == SYS_RUNNING)
    {
        uint32_t total = ctx->run_time_sec + (now - ctx->run_start_tick) / 1000U;
        if (total != s_prev_display_sec)
        {
            s_prev_display_sec = total;
            ctx->dirty_mask   |= DIRTY_TIME;
        }
    }
}

/* ================================================================
 *  IR Counter
 * ================================================================ */

static volatile uint32_t s_last_ir_tick = 0;
#define IR_DEBOUNCE_MS  50

void IR_Counter_Callback(MachineContext_t *ctx)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ir_tick) < IR_DEBOUNCE_MS) return;
    s_last_ir_tick = now;
    ctx->peel_count++;
    ctx->dirty_mask |= DIRTY_COUNT;
}
