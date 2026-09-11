#ifndef __MAIN_TASK_H__
#define __MAIN_TASK_H__

#define 	__PRELIMINARY_ROUND__       //使用初赛代码

#include "main.h"
#include "Output_task.h"
#include "Chassis_task.h"
#include "Tower_task.h"

extern struct RC_DATA RC_RX;
extern uint8_t GET_TASK_FLAGE;

extern void Main_task(void);
extern void Main_init(void);

#endif
