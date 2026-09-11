/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   底盘控制 H 文件
  ******************************************************************************
  * @attention
  * 邮箱：2671780885@qq.com
  ******************************************************************************
  */

#include "main.h"
#include "tim.h"
#include "usart.h"

#include "Output_task.h"
#include <stdlib.h>
#include "math.h"

extern int SpeedTarget[4];
extern float pos_x;
extern float pos_y;
extern float zangle;
extern uint8_t delay_pos;
extern uint8_t delay_visual;
 
void SpeedTarget_stop(void);                                                                             //电机停转
void SetMotorVoltageAndDirection(int MotorSpeed1, int MotorSpeed2, int MotorSpeed3, int MotorSpeed4);    //电机电压和方向控制函数
void chassis_RC(short x,short y,short z);   																													   //底盘遥控函数
void chassis_move(int x,int y,int z);      																															 //底盘移动函数
void chassis_turn(int z);   																																						 //底盘转动函数
void visual_pos_adj(uint8_t target,int x,int y,int z);                                                   //底盘微调函数（目标号，目标x，目标y，保持角度）
void z_adj(int z);  																																										 //底盘偏航微调
