#ifndef __RM3508_H
#define __RM3508_H

#include "main.h"
#include "can.h"

/* 电机数量：支持 0x201 ~ 0x204 */
#define MOTOR_NUM  4

/* RM3508/C620 反馈 ID 范围 */
#define RM3508_FEEDBACK_ID_BASE  0x201
#define RM3508_FEEDBACK_ID_MAX   (RM3508_FEEDBACK_ID_BASE + MOTOR_NUM - 1)

/* CAN 发送ID：
 * 0x200 -> 0x201 ~ 0x204
 * 0x1FF -> 0x205 ~ 0x208
 */
#define CAN_TX_ID_GROUP1  0x200
#define CAN_TX_ID_GROUP2  0x1FF

/* PID 参数 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_output;
    float integral_limit;
    float deadband;
    float target;
    float measure;
    float err;
    float last_err;
    float pout;
    float iout;
    float dout;
    float output;
} PID_t;

/* 电机反馈数据 */
typedef struct {
    uint16_t ecd;           // 编码器值 0~8191
    int16_t  speed_rpm;     // 转速 RPM
    int16_t  given_current; // 实际转矩电流
    uint8_t  temperature;   // 温度
} Motor_Measure_t;

/* 初始化：CAN滤波器 + 启动CAN + 使能中断 */
void RM3508_Init(void);

/* PID 初始化 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float integral_limit, float deadband);

/* PID 计算 */
float PID_Calc(PID_t *pid, float target, float measure);

/* 发送四路电流指令到 0x200（对应 0x201 ~ 0x204） */
void RM3508_SendCurrents(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/* 发送四路电流指令到 0x1FF（对应 0x205 ~ 0x208） */
void RM3508_SendCurrentsGroup2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

/* 兼容旧测试代码：发送前两路电流，后两路自动补 0 */
void RM3508_SendCurrent(int16_t motor1, int16_t motor2);

/* 发送单路测试电流，其余路自动补 0 */
void RM3508_SendCurrentSingle(uint8_t motor_id, int16_t current);

/* 设置电机速度（RPM），内部做PID */
void RM3508_SetSpeed(uint8_t motor_id, int16_t speed_rpm);

/* 停止电机 */
void RM3508_Stop(uint8_t motor_id);

/* 停止所有电机 */
void RM3508_StopAll(void);

/* 在主循环中调用，执行PID计算并发送指令 */
void RM3508_Control_Loop(void);
void RM3508_Service(void);

/* 获取电机反馈数据 */
Motor_Measure_t* RM3508_GetMotorData(uint8_t motor_id);

/* 调试观察变量：用于在线调试看 CAN 收发状态 */
extern volatile uint32_t g_rm3508_last_can_id;
extern volatile int16_t  g_rm3508_last_speed_rpm;
extern volatile uint16_t g_rm3508_last_ecd;
extern volatile uint8_t  g_rm3508_last_temperature;
extern volatile uint32_t g_rm3508_rx_count;
extern volatile uint32_t g_rm3508_tx_count;
extern volatile uint32_t g_rm3508_tx_fail_count;
extern volatile uint32_t g_rm3508_last_error_code;
extern volatile uint32_t g_rm3508_init_stage;
extern volatile uint8_t  g_rm3508_init_ok;

#endif /* __RM3508_H */
