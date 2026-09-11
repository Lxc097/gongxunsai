/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   视觉接收端
  ******************************************************************************
  * @attention
  * 邮箱：2671780885@qq.com
  ******************************************************************************
  */

#ifndef VISUAL_IDENTITY_H
#define VISUAL_IDENTITY_H

#include "main.h"
#include "string.h"
#include "Main_task.h"

struct VISUAL_DATA
{
  short r_block_x,r_block_y,
        g_block_x,g_block_y,
        b_block_x,b_block_y,
	      rgb_circle_x,
	      rgb_circle_y,
	      rgb_circle_z,   //左环减右环
	      tray_center_x,
	      tray_center_y;
	uint8_t block_grab;
	uint8_t block_order;   //决赛物料摆放顺序
};

extern struct VISUAL_DATA VIS_RX;  //视觉接收数据
extern uint8_t color_task[6];

void vsiual_data_inti(void);    //接收初始化
void visual_idle(void);    //空闲
void identify_QR_code(void);    //识别二维码
void identify_material_tray(void);    //识别物料盘
void identify_materials(uint8_t num);    //识别物料 颜色1/2/3红绿蓝
void request_grab_materials(uint8_t num);    //请求抓取物料 颜色1/2/3红绿蓝
void identify_color_rings(void);    //色环位置和z轴偏差
void stacking_positioning(void);    //码垛时色环位置和z轴偏差
void identify_material_tray_final(void);    //决赛物料盘识别
void request_to_place_materials(void);    //决赛请求放置物料 颜色1/2/3红绿蓝

void OPENMV_Data_Unpack(uint8_t openmv_data[16]);     //OPENMV接收解包
void NANO_Data_Unpack(uint8_t nano_data[16]);     //NANO接收解包

#endif
