#ifndef __DC_MOTOR_IR_TEST_H
#define __DC_MOTOR_IR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool DcMotorIrTest_IsActive(void);
void DcMotorIrTest_Init(void);
void DcMotorIrTest_Process(void);
void DcMotorIrTest_IrCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __DC_MOTOR_IR_TEST_H */
