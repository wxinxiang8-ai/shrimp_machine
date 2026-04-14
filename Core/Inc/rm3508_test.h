#ifndef __RM3508_TEST_H
#define __RM3508_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Rm3508Test_IsActive(void);
void Rm3508Test_Init(void);
void Rm3508Test_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __RM3508_TEST_H */
