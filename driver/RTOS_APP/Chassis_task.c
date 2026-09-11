/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   底盘主任务
  ******************************************************************************
  * @attention
  * 邮箱：sakura.mail@qq.com
  ******************************************************************************
  */

#include "Chassis_task.h"
#include "Tower_task.h"
#include "dynamic_map_link.h"
#include "zdt_motor.h"

uint8_t chassis_mode=2;    //底盘模式
uint8_t control_mode=DYNAMIC_MAP_NAVIGATION_ONLY ? 2U : 0U;

int X_target = 0, 
	  Y_target = 0, 
    Z_target = 0;

float d_X = 0,
	    d_Y = 0;

void Motor_Init(void)    //底盘初始化
{
#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN && !ZDT_CAN_COMMISSIONED
  /* Commissioning-locked images must keep the wheel power rail disabled. */
  VM_OFF();
#else
  VM_ON();
#endif
	visual_idle();    //视觉空闲
	SpeedTarget_stop();
	DynamicMap_Link_Init();
	osDelay(100); /* Motor-side power settling, before the first bus command. */
	ZdtMotor_Init();
	if(ZdtMotor_GetFault() != ZDT_FAULT_NONE)
		DynamicMap_Link_ReportMotorFault();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		底盘目标位置更新
//  @return     			/
//  @param					 x 位置（mm）
//  @param					 y 位置（mm）		
//  @param					 z 位置（mm）   
//  @Sample usage:    /
//-------------------------------------------------------------------------------------------------------------------
void Chassis_target_updata(int x, int y, int z)    //底盘目标位置更新
{
  X_target = x;
	Y_target = y;
	Z_target = z;
	chassis_mode = 1;    //开启场地位置环
	in_pos = 0;
	near_pos = 0;
	delay_pos = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		底盘目标定位
//  @return     			/
//  @param					 x 位置（mm）
//  @param					 y 位置（mm）		
//  @param					 z 位置（deg）
//  @param					 precise 二次修正 0:修正（默认） 1:不修正	
//  @param					 delay 到位延时（ms）
//  @Sample usage:    /
//-------------------------------------------------------------------------------------------------------------------
void Chassis_Go_Pos(int x, int y, int z, uint8_t precise, uint16_t delay)    //底盘目标定位
{
  	Chassis_target_updata(x, y, z);
	  if (precise==1)
	    while(near_pos == 0) {osDelay(10);}  //接近
	  else
	    while(in_pos == 0) {osDelay(10);}  //就位
		
		if (delay!=0)
		  osDelay(delay);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		底盘目标位置逼近值清零（临时，退出后恢复上次绝对位置）
//  @return     			/
//  @param					  /
//  @Sample usage:    /
//-------------------------------------------------------------------------------------------------------------------
void Chassis_d_zero(void)    //底盘目标位置逼近值清零
{
	d_X = 0;
	d_Y = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		底盘视觉定位
//  @return     			/
//  @param					 目标编号 1物料盘 2色环 3码垛时色环
//  @param					 x 位置（像素）
//  @param					 y 位置（像素）		
//  @param					 z 位置（像素差）
//  @param					 delay 到位延时（ms）
//  @Sample usage:    /
//-------------------------------------------------------------------------------------------------------------------
void Chassis_Visual_Pos(uint8_t target, int x, int y, int z, uint16_t delay)    //底盘视觉定位
{
	chassis_mode = 2;    //解除场地位置环
	
//	identify_posture(1500);  //识别姿态
	
	LED_2_ON();
  v_in_pos = 0;
	delay_visual=0;
	vsiual_data_inti();    //清除接收数据
	SpeedTarget_stop();    //电机停转
	short target_data = 0;
	
	while(target_data==0)
	{
    switch(target)    //请求获取对应目标位置
		{
    case 1:  //物料盘
			  identify_material_tray();
		    target_data = VIS_RX.tray_center_x + VIS_RX.tray_center_y;
        break;
    case 2:  //色环
			  identify_color_rings();
		    target_data = VIS_RX.rgb_circle_x + VIS_RX.rgb_circle_y;
        break;
    case 3:  //码垛时色环
			  stacking_positioning();
		    target_data = VIS_RX.rgb_circle_x + VIS_RX.rgb_circle_y;
        break;
		case 4:  //色环粗校准
			  identify_color_rings();
		    target_data = VIS_RX.rgb_circle_x + VIS_RX.rgb_circle_y;
        break;
	  }
		  osDelay(100);
	}
	    while(v_in_pos == 0) {visual_pos_adj(target,x,y,z); osDelay(5);}  //就位
//			while(1) {visual_pos_adj(target,x,y,z); osDelay(5);}  //就位
			visual_idle();    //空闲
			LED_2_OFF();
			SpeedTarget_stop();    //电机停转
			buzzer_rings(20, 20, 3);
		
		if (delay!=0)
		  osDelay(delay);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		底盘主任务
//  @return     			/
//  @param					  /
//  @Sample usage:    /
//-------------------------------------------------------------------------------------------------------------------
void Chassis_task(void)    //底盘任务
{
  Motor_Init();
	osDelay(100);
#if !DYNAMIC_MAP_NAVIGATION_ONLY
	while(!ready) { osDelay(10);}
#endif
	
	while(1)
	{
		int16_t map_x = 0, map_y = 0, map_z = 0;
#if DYNAMIC_MAP_NAVIGATION_ONLY
		control_mode = 2U;
#endif
		DynamicMap_Link_Process();
		if(control_mode==0)
		{
	    switch(chassis_mode)
	  	{
		    case 1:
		  		chassis_move(X_target + d_X, Y_target + d_Y, Z_target);    //底盘定位
		  	  break;
	  		case 2:
				  if (v_in_pos == 1)
            SpeedTarget_stop();    //电机停转
					osDelay(5);
		  		break;
		  }
	  }
		else if(control_mode==1)
		{
				chassis_RC(RC_RX.LX,RC_RX.LY,RC_RX.RX);     //底盘遥控
		}
		else if(control_mode==2)
		{
			if(DynamicMap_Link_GetVelocity(&map_x, &map_y, &map_z) != 0U)
				chassis_RC(map_x, map_y, map_z);
			else
				SpeedTarget_stop();
		}
		SetMotorVoltageAndDirection(SpeedTarget[0], SpeedTarget[1], SpeedTarget[2], SpeedTarget[3]);
		osDelay(5);
	}
}
