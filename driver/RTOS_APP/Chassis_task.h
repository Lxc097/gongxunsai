/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   底盘主任务
  ******************************************************************************
  * @attention
  * 邮箱：2671780885@qq.com
  ******************************************************************************
  */

#ifndef __CHASSIS_TASK__
#define __CHASSIS_TASK__

#include "main.h"

extern uint8_t ready;
extern uint8_t in_pos;    //就位指示
extern uint8_t v_in_pos;    //视觉就位指示
extern uint8_t near_pos;    //接近指示
extern float d_X,
	           d_Y;
extern uint8_t chassis_mode;    //底盘模式
extern uint8_t control_mode;    //遥控模式

void Motor_Init(void);    //底盘初始化
void Chassis_target_updata(int x, int y, int z);    //底盘目标位置更新
void Chassis_d_zero(void);    //底盘目标位置逼近值清零
void Chassis_Go_Pos(int x, int y, int z, uint8_t precise, uint16_t delay);    //底盘目标定位
void Chassis_Visual_Pos(uint8_t target, int x, int y, int z, uint16_t delay);    //底盘视觉定位
void Chassis_task(void);    //底盘任务

#endif  /* __CHASSIS_TASK__ */
