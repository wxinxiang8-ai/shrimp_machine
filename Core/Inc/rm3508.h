#ifndef __RM3508_H
#define __RM3508_H

#include "main.h"
#include "can.h"

/* 电机数量 */
#define MOTOR_NUM  2

/* CAN 发送ID */
#define CAN_TX_ID  0x200

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

/* 发送电流指令给两个电机 */
void RM3508_SendCurrent(int16_t motor1, int16_t motor2);

/* 设置电机速度（RPM），内部做PID */
void RM3508_SetSpeed(uint8_t motor_id, int16_t speed_rpm);

/* 停止电机 */
void RM3508_Stop(uint8_t motor_id);

/* 停止所有电机 */
void RM3508_StopAll(void);

/* 在主循环中调用，执行PID计算并发送指令 */
void RM3508_Control_Loop(void);

/* 获取电机反馈数据 */
Motor_Measure_t* RM3508_GetMotorData(uint8_t motor_id);

#endif /* __RM3508_H */
