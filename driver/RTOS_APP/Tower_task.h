/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   塔吊主任务
  ******************************************************************************
  * @attention
  * 邮箱：2671780885@qq.com
  ******************************************************************************
  */

#ifndef __TOWER_TASK__
#define __TOWER_TASK__

#include "main.h"

extern uint8_t ready;
extern uint8_t in_pos;    //就位指示
extern uint8_t near_pos;    //接近指示

void OS_Tower_Init(void);    //塔吊初始化
void Tower_target_updata(int radius, int height, int theta);    //塔吊目标位置更新
void Tower_d_zero(void);    //塔吊目标位置逼近值清零
void Tower_task(void);    //塔吊任务

void reset_posture(void);     //复位姿态
void identify_posture(int radius, int height);     //识别姿态

void grab_turntable_B(uint8_t color, uint8_t num);     //物料台动态抓取
void track_materials(uint8_t color);     //物料跟踪

void grab_materials_ground(uint8_t color ,uint8_t first);     //地面抓物料
void put_materials_car(uint8_t order);     //车上放置物料
void put_materials_car_right(uint8_t order);     //车上放置物料左边
void grab_materials_car(uint8_t order, uint8_t first);     //车上抓取物料
void put_materials_ground(uint8_t color, uint16_t stack);     //地面放物料

void put_block_turntable(uint8_t order, uint8_t color);     //决赛物料台放置
void put_turntable_action(uint8_t order, uint8_t pos);     //决赛物料台放置动作

#endif  /* __TOWER_TASK__ */
