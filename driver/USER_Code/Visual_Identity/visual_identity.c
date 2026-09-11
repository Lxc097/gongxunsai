/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   视觉接收端
  ******************************************************************************
  * @attention
  * 邮箱：sakura.mail@qq.com
  ******************************************************************************
  */

#include "visual_identity.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      接收初始化
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void vsiual_data_inti(void)    //接收初始化
{
  VIS_RX.r_block_x = 0;
	VIS_RX.r_block_y = 0;
	VIS_RX.g_block_x = 0;
	VIS_RX.g_block_y = 0;
	VIS_RX.b_block_x = 0;
	VIS_RX.b_block_y = 0;
	VIS_RX.rgb_circle_x = 0; 
	VIS_RX.rgb_circle_y = 0;
	VIS_RX.rgb_circle_z = 0;  //左环减右环
	VIS_RX.tray_center_x = 0;
	VIS_RX.tray_center_y = 0;
	VIS_RX.block_grab = 0;    //抓取指令
	VIS_RX.block_order = 0;   //决赛物料摆放顺序
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      空闲
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void visual_idle(void)    //空闲
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x00;      //任务码 0 空闲
  nano_data[2]=0x00;      //识别目标
  nano_data[3]=0x00;      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      识别二维码
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void identify_QR_code(void)    //识别二维码
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x09;      //任务码 0 空闲
  nano_data[2]=0x00;      //识别目标
  nano_data[3]=0x00;      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      识别物料盘
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void identify_material_tray(void)    //识别物料盘
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x01;      //任务码 1 识别物料盘
  nano_data[2]=0x00;      //识别目标
  nano_data[3]=nano_data[1]+nano_data[2];      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      识别物料 颜色1/2/3红绿蓝
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void identify_materials(uint8_t num)    //识别物料 颜色1/2/3红绿蓝
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x02;      //任务码 2 识别物料
  nano_data[2]=num;      //识别目标
  nano_data[3]=nano_data[1]+nano_data[2];      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求抓取物料 颜色1/2/3红绿蓝
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void request_grab_materials(uint8_t num)    //请求抓取物料 颜色1/2/3红绿蓝
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x07;      //任务码 2 识别物料
  nano_data[2]=num;      //识别目标
  nano_data[3]=nano_data[1]+nano_data[2];      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      色环位置和z轴偏差
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void identify_color_rings(void)   //色环位置和z轴偏差
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x03;      //任务码 3 识别色环
  nano_data[2]=0x00;      //识别目标
  nano_data[3]=nano_data[1]+nano_data[2];      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      码垛时色环位置和z轴偏差
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void stacking_positioning(void)    //码垛时色环位置和z轴偏差
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x08;      //任务码 4 码垛识别物料
  nano_data[2]=0x00;      //识别目标
  nano_data[3]=nano_data[1]+nano_data[2];      //和校验

HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      决赛物料盘识别
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void identify_material_tray_final(void)    //决赛物料盘识别
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x05;      //任务码 5 决赛物料盘识别
  nano_data[2]=0x00;      //识别目标
  nano_data[3]=nano_data[1]+nano_data[2];      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      决赛请求放置物料 颜色1/2/3红绿蓝
//  @return     /
//  @param			/
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void request_to_place_materials(void)    //决赛请求放置物料 颜色1/2/3红绿蓝
{
  uint8_t nano_data[4];    //定义数据包数组
	
  nano_data[0]=0x66;      //帧头1
  nano_data[1]=0x06;      //任务码 6 决赛请求放置物料
  nano_data[2]=0x00;      //识别目标
  nano_data[3]=nano_data[1]+nano_data[2];      //和校验
	
HAL_UART_Transmit(&huart1,nano_data,4,HAL_MAX_DELAY);    //数据发送
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 OPENMV接收解包
//  @return     		 /
//  @param					 uint8_t openmv_data[16]  传入接收到的数据包
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void OPENMV_Data_Unpack(uint8_t openmv_data[16])     //OPENMV接收解包
{
    	//和校验：计算所有数据帧之和
    	if (openmv_data[0]==0x88 && (openmv_data[1] + openmv_data[2] + openmv_data[3] + 
				                           openmv_data[4] + openmv_data[5] + openmv_data[6]) == 12)    //校验帧头，和校验
  	  {
				for (uint8_t i=0;i<6;i++)
          color_task[i]=openmv_data[i+1];
	  	}      
	    else
	    {
				for (uint8_t i=0;i<6;i++)
		  	  color_task[i]=0;
	    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 NANO接收解包
//  @return     		 /
//  @param					 uint8_t nano_data[16]  传入接收到的数据包
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void NANO_Data_Unpack(uint8_t nano_data[16])     //NANO接收解包
{
	if (nano_data[0]!=0x66)    //校验帧头
		return;
	switch(nano_data[1])
	{
		case 1:    //物料盘位置
			if (nano_data[6]!=0x77)    //校验帧尾
				return;
			VIS_RX.tray_center_x = (nano_data[2]<<8)|nano_data[3];
			VIS_RX.tray_center_y = (nano_data[4]<<8)|nano_data[5];
			break;
			
		case 2:    //物料位置
			if (nano_data[7]!=0x77)    //校验帧尾
		    return;
			switch (nano_data[2])
			{
			case 1:    //R位置
				VIS_RX.r_block_x = (nano_data[3]<<8)|nano_data[4];
			  VIS_RX.r_block_y = (nano_data[5]<<8)|nano_data[6];
				break;
			case 2:    //G位置
				VIS_RX.g_block_x = (nano_data[3]<<8)|nano_data[4];
			  VIS_RX.g_block_y = (nano_data[5]<<8)|nano_data[6]; 
				break;
			case 3:    //B位置
				VIS_RX.b_block_x = (nano_data[3]<<8)|nano_data[4];
			  VIS_RX.b_block_y = (nano_data[5]<<8)|nano_data[6];
				break;
			default:
			  break;
		  }
			break;
			
		case 3:    //色环位置和z轴偏差
			if (nano_data[8]!=0x77)    //校验帧尾
		    return;
			VIS_RX.rgb_circle_x = (nano_data[2]<<8)|nano_data[3];
			VIS_RX.rgb_circle_y = (nano_data[4]<<8)|nano_data[5];
			VIS_RX.rgb_circle_z = (short)((nano_data[6]<<8)|nano_data[7]);
			break;
		
		case 4:    //码垛时色环位置和z轴偏差
			if (nano_data[8]!=0x77)    //校验帧尾
		    return;
			VIS_RX.rgb_circle_x = (nano_data[2]<<8)|nano_data[3];
			VIS_RX.rgb_circle_y = (nano_data[4]<<8)|nano_data[5];
			VIS_RX.rgb_circle_z = (short)((nano_data[6]<<8)|nano_data[7]);
			break;
		
		case 5:    //决赛物料盘位置
			break;
		
		case 6:    //决赛物料放置指令
			if (nano_data[3]!=0x77)    //校验帧尾
		    return;
			VIS_RX.block_order = nano_data[2];
			break;
		
		case 7:    //初赛方案2抓取物料指令
			if (nano_data[3]!=0x77)    //校验帧尾
		    return;
			VIS_RX.block_grab = nano_data[2];
			break;
			
		case 8:    //决赛抓取物料校准和放置顺序
			if (nano_data[9]!=0x77)    //校验帧尾
		    return;
			VIS_RX.rgb_circle_x = (nano_data[2]<<8)|nano_data[3];
			VIS_RX.rgb_circle_y = (nano_data[4]<<8)|nano_data[5];
			VIS_RX.rgb_circle_z = (short)((nano_data[6]<<8)|nano_data[7]);
			VIS_RX.block_order = nano_data[8];
			break;
		
		case 9:    //二维码接收
			if (nano_data[8]!=0x77 || !GET_TASK_FLAGE)    //校验帧尾
		    return;
			//和校验：计算所有数据帧之和
    	if (nano_data[2] + nano_data[3] + nano_data[4] + 
				  nano_data[5] + nano_data[6] + nano_data[7] == 12)    //校验帧头，和校验
  	  {
				for (uint8_t i=0;i<6;i++)
          color_task[i]=nano_data[i+2];
	  	}      
	    else
	    {
				for (uint8_t i=0;i<6;i++)
		  	  color_task[i]=0;
	    }
			break;
		
		case 0xAA:    //调试
			if (nano_data[3]!=0x77)    //校验帧尾
		    return;
			if (nano_data[2]==1)
				Cam_Light_ON();
			else if (nano_data[2]==2)
        Cam_Light_OFF();
			else if (nano_data[2]==3)
        LED_2_ON();
			break;
			
		default:
			break;
	}       	                
}
