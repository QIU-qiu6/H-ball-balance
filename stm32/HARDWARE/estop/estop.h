#ifndef __ESTOP_H__
#define __ESTOP_H__

#include "sys.h"

/* PA0 active-low emergency-stop input. The latch clears only on reset. */
void Estop_Init(void);
void Estop_Tick1ms(void);
u8 Estop_IsLatched(void);

#endif
