#ifndef __DUMP_H
#define __DUMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "main.h"

void Dump_Init(void);
void Dump_Start(void);
void Dump_Stop(void);
void Dump_SetState(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* __DUMP_H */
