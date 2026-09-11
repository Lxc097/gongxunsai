#ifndef __OUTPUT_H__
#define __OUTPUT_H__

#include "main.h"

extern uint8_t ready;
extern struct RC_DATA RC_RX;
extern void Output_task(void);
extern void Output_init(void);

#endif
