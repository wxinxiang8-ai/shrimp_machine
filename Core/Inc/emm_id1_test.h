#ifndef __EMM_ID1_TEST_H
#define __EMM_ID1_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

bool EmmId1Test_IsActive(void);
void EmmId1Test_Init(void);
void EmmId1Test_Process(void);
void EmmId1Test_IrCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __EMM_ID1_TEST_H */
