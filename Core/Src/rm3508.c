/*
 * @Author: Xiang xin wang wxinxiang8@gmail.com
 * @Description: RM3508 电机控制 - CAN通信 + PID速度环
 */
#include "rm3508.h"
#include <string.h>

/* 电机反馈数据 */
static Motor_Measure_t motor_data[MOTOR_NUM];

/* 调试观察变量：用于在线调试看 CAN 收发状态 */
volatile uint32_t g_rm3508_last_can_id = 0;
volatile int16_t  g_rm3508_last_speed_rpm = 0;
volatile uint16_t g_rm3508_last_ecd = 0;
volatile uint8_t  g_rm3508_last_temperature = 0;
volatile uint32_t g_rm3508_rx_count = 0;
volatile uint32_t g_rm3508_tx_count = 0;
volatile uint32_t g_rm3508_tx_fail_count = 0;
volatile uint32_t g_rm3508_last_error_code = 0;
volatile uint32_t g_rm3508_init_stage = 0;
volatile uint8_t  g_rm3508_init_ok = 0;

/* PID 控制器 */
static PID_t motor_pid[MOTOR_NUM];

/* 目标速度 */
static int16_t target_speed[MOTOR_NUM] = {0, 0, 0, 0};

/* 电机使能标志 */
static uint8_t motor_enabled[MOTOR_NUM] = {0, 0, 0, 0};

/* CAN 发送/接收相关 */
static CAN_TxHeaderTypeDef tx_header;
static uint8_t tx_data[8];
static uint32_t tx_mailbox;

/*-----------------------------------------------------------
 * CAN 滤波器初始化 + 启动 + 使能接收中断
 *-----------------------------------------------------------*/
void RM3508_Init(void)
{
    /* 配置CAN滤波器 - 接收所有ID */
    CAN_FilterTypeDef filter;

    g_rm3508_init_stage = 1;
    g_rm3508_init_ok = 0;
    g_rm3508_last_error_code = HAL_CAN_ERROR_NONE;

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    g_rm3508_init_stage = 2;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        g_rm3508_last_error_code = HAL_CAN_GetError(&hcan1);
        return;
    }

    g_rm3508_init_stage = 3;
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        g_rm3508_last_error_code = HAL_CAN_GetError(&hcan1);
        return;
    }

    g_rm3508_init_stage = 4;
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        g_rm3508_last_error_code = HAL_CAN_GetError(&hcan1);
        return;
    }

    g_rm3508_init_stage = 5;
    /* 初始化所有电机的PID: Kp=10, Ki=0.02, Kd=0 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        PID_Init(&motor_pid[i], 10.0f, 0.02f, 0.0f, 16384.0f, 5000.0f, 10.0f);
    }

    /* 清零反馈数据和控制状态 */
    memset(motor_data, 0, sizeof(motor_data));
    memset(target_speed, 0, sizeof(target_speed));
    memset(motor_enabled, 0, sizeof(motor_enabled));
    g_rm3508_last_can_id = 0;
    g_rm3508_last_speed_rpm = 0;
    g_rm3508_last_ecd = 0;
    g_rm3508_last_temperature = 0;
    g_rm3508_rx_count = 0;
    g_rm3508_tx_count = 0;
    g_rm3508_tx_fail_count = 0;
    g_rm3508_last_error_code = HAL_CAN_ERROR_NONE;
    g_rm3508_init_stage = 6;
    g_rm3508_init_ok = 1;
}

/*-----------------------------------------------------------
 * PID 初始化
 *-----------------------------------------------------------*/
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float integral_limit, float deadband)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_out;
    pid->integral_limit = integral_limit;
    pid->deadband = deadband;
    pid->target = 0;
    pid->measure = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->output = 0;
}

/*-----------------------------------------------------------
 * PID 计算
 *-----------------------------------------------------------*/
float PID_Calc(PID_t *pid, float target, float measure)
{
    pid->target = target;
    pid->measure = measure;
    pid->err = target - measure;

    /* 死区判断 */
    if (pid->err > -pid->deadband && pid->err < pid->deadband) {
        pid->err = 0;
    }

    pid->pout = pid->kp * pid->err;
    pid->iout += pid->ki * pid->err;
    pid->dout = pid->kd * (pid->err - pid->last_err);

    /* 积分限幅 */
    if (pid->iout > pid->integral_limit)  pid->iout = pid->integral_limit;
    if (pid->iout < -pid->integral_limit) pid->iout = -pid->integral_limit;

    pid->output = pid->pout + pid->iout + pid->dout;

    /* 输出限幅 */
    if (pid->output > pid->max_output)  pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    pid->last_err = pid->err;
    return pid->output;
}

static void RM3508_SendCurrentsRaw(uint32_t std_id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = (uint8_t)(motor1 >> 8);
    tx_data[1] = (uint8_t)(motor1 & 0xFF);
    tx_data[2] = (uint8_t)(motor2 >> 8);
    tx_data[3] = (uint8_t)(motor2 & 0xFF);
    tx_data[4] = (uint8_t)(motor3 >> 8);
    tx_data[5] = (uint8_t)(motor3 & 0xFF);
    tx_data[6] = (uint8_t)(motor4 >> 8);
    tx_data[7] = (uint8_t)(motor4 & 0xFF);

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        g_rm3508_tx_fail_count++;
        g_rm3508_last_error_code = HAL_CAN_GetError(&hcan1);
        return;
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        g_rm3508_tx_fail_count++;
        g_rm3508_last_error_code = HAL_CAN_GetError(&hcan1);
        return;
    }

    g_rm3508_tx_count++;
    g_rm3508_last_error_code = HAL_CAN_ERROR_NONE;
}

/*-----------------------------------------------------------
 * 发送四路电流指令 (CAN ID 0x200 -> 0x201 ~ 0x204)
 *-----------------------------------------------------------*/
void RM3508_SendCurrents(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    RM3508_SendCurrentsRaw(CAN_TX_ID_GROUP1, motor1, motor2, motor3, motor4);
}

void RM3508_SendCurrentsGroup2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    RM3508_SendCurrentsRaw(CAN_TX_ID_GROUP2, motor5, motor6, motor7, motor8);
}

void RM3508_SendCurrent(int16_t motor1, int16_t motor2)
{
    RM3508_SendCurrents(motor1, motor2, 0, 0);
}

void RM3508_SendCurrentSingle(uint8_t motor_id, int16_t current)
{
    int16_t output_group1[MOTOR_NUM] = {0};
    int16_t output_group2[MOTOR_NUM] = {0};

    if (motor_id < MOTOR_NUM) {
        output_group1[motor_id] = current;
    } else if (motor_id < (MOTOR_NUM * 2U)) {
        output_group2[motor_id - MOTOR_NUM] = current;
    } else {
        return;
    }

    RM3508_SendCurrents(output_group1[0], output_group1[1], output_group1[2], output_group1[3]);
    RM3508_SendCurrentsGroup2(output_group2[0], output_group2[1], output_group2[2], output_group2[3]);
}

/*-----------------------------------------------------------
 * 设置电机目标速度
 *-----------------------------------------------------------*/
void RM3508_SetSpeed(uint8_t motor_id, int16_t speed_rpm)
{
    if (motor_id >= MOTOR_NUM) return;
    target_speed[motor_id] = speed_rpm;
    motor_enabled[motor_id] = 1;
}

/*-----------------------------------------------------------
 * 停止单个电机
 *-----------------------------------------------------------*/
void RM3508_Stop(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return;
    target_speed[motor_id] = 0;
    motor_enabled[motor_id] = 0;
    motor_pid[motor_id].iout = 0;  /* 清积分 */
}

/*-----------------------------------------------------------
 * 停止所有电机
 *-----------------------------------------------------------*/
void RM3508_StopAll(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        RM3508_Stop(i);
    }
    RM3508_SendCurrents(0, 0, 0, 0);
}

/*-----------------------------------------------------------
 * 主循环调用：PID计算 + 发送指令
 *-----------------------------------------------------------*/
void RM3508_Control_Loop(void)
{
    int16_t output[MOTOR_NUM] = {0};

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        if (motor_enabled[i]) {
            PID_Calc(&motor_pid[i], (float)target_speed[i], (float)motor_data[i].speed_rpm);
            output[i] = (int16_t)motor_pid[i].output;
        }
    }

    RM3508_SendCurrents(output[0], output[1], output[2], output[3]);
}

/*-----------------------------------------------------------
 * 获取电机反馈数据
 *-----------------------------------------------------------*/
Motor_Measure_t* RM3508_GetMotorData(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return &motor_data[0];
    return &motor_data[motor_id];
}

/*-----------------------------------------------------------
 * CAN 接收回调 - 解析C620电调反馈
 * C620 反馈ID: 0x201 ~ 0x204
 *-----------------------------------------------------------*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance != CAN1) {
        return;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        g_rm3508_last_error_code = HAL_CAN_GetError(hcan);
        return;
    }

    g_rm3508_last_can_id = rx_header.StdId;
    g_rm3508_last_ecd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
    g_rm3508_last_speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);
    g_rm3508_last_temperature = rx_data[6];
    g_rm3508_rx_count++;

    /* 判断是否为电机反馈 (0x201 ~ 0x204) */
    if (rx_header.StdId >= RM3508_FEEDBACK_ID_BASE && rx_header.StdId <= RM3508_FEEDBACK_ID_MAX) {
        uint8_t idx = (uint8_t)(rx_header.StdId - RM3508_FEEDBACK_ID_BASE);
        motor_data[idx].ecd           = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
        motor_data[idx].speed_rpm     = (int16_t)(rx_data[2] << 8 | rx_data[3]);
        motor_data[idx].given_current = (int16_t)(rx_data[4] << 8 | rx_data[5]);
        motor_data[idx].temperature   = rx_data[6];
    }
}
