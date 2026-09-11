/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2034-08-08
  * @brief   接收端 H 文件
  ******************************************************************************
  * @attention
  * 邮箱：2671780885@qq.com
  ******************************************************************************
  */
  
  /**
  ******************************************************************************
  使用方法：
  1.CUBEMX设置串口中断 在.h文件第45行修改串口号
  2.包含头文件 位于USER CODE BEGIN Includes
    #include "Remote_Control_RX"
  3.定义串口中断缓冲区和接收结构体 位于USER CODE BEGIN PV
    uint8_t RX_DATA;
    struct RC_DATA RC_RX;
  4.使能串口中断 位于USER CODE BEGIN 2
    HAL_UART_Receive_IT(RC_RX_huart,&RX_DATA,1); 
  5.添加串口中断回调函数 位于USER CODE BEGIN 4
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
        if(huart == RC_RX_huart)    //串口1中断  RC接收
        {
	    	RC_Reception(RX_DATA);
	    	//HAL_GPIO_TogglePin(GPIOX,GPIO_PIN_X);   //串口接收指示灯
  	        HAL_UART_Receive_IT(RC_RX_huart,&RX_DATA,1); 
        }
    }
  ******************************************************************************
  */

#ifndef REMOTE_CONTROL_RX
#define REMOTE_CONTROL_RX

#include "main.h"
#include "string.h"

struct RC_DATA
{
	// int8_t 是有符号char型 -128 到 127
	int8_t LX;
	int8_t LY;
	int8_t RX;
	int8_t RY;

	// 2个旋钮
	int8_t KNOB_L;
	int8_t KNOB_R;

	// 4个前端按键
	int8_t KEY_L1;
	int8_t KEY_L2;
  int8_t KEY_R1;
  int8_t KEY_R2;

	// 4个拨杆开关
  int8_t TOG_L1;
  int8_t TOG_L2;
  int8_t TOG_R1;
  int8_t TOG_R2;
		
	int8_t RC_CONNECT_FLAG;
		
};

extern struct RC_DATA RC_RX;
extern uint8_t control_mode;    //遥控模式

void RC_Data_Unpack(uint8_t rc_data[16]);    //DMA解包

#endif
