/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   输入输出任务
  ******************************************************************************
  * @attention
  * 邮箱：sakura.mail@qq.com
  ******************************************************************************
  */

#include "Output_task.h"

uint8_t ready = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		输出初始化
//  @return     			/
//  @param					  /
//  @Sample usage:    /
//-------------------------------------------------------------------------------------------------------------------
void Output_init(void)
{
	LED_1_OFF();
	LED_2_OFF();
	LED_3_OFF();
	Buzzer_OFF();
	
	SPI_OLED_Init();
	SPI_OLED_Clear();
	CAM_Light_OFF();
}

void Output_task(void)
{
	Output_init();
	SPI_OLED_ShowString(16,0,(char *)"SW Ver 2.7.4",16,0);
	SPI_OLED_ShowString(8,2,(char *)"System init ",16,0);
	buzzer_rings(20, 50, 2);
	
	for(uint16_t i=1400;i>0&&ready==0;i--)
	{
		if(i%100==0)
		 SPI_OLED_ShowNum(104,2,i/100 - 1,2,16,0);
		osDelay (10);
		if (HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin))
			break;
	}
	ready = 1;
	SPI_OLED_ShowString(0,2,(char *)"      Reday     ",16,0);	
	
	while(1)
	{

		SPI_OLED_Showdecimal(16,4,pos_x,6,1,16,0);
		SPI_OLED_Showdecimal(16,6,pos_y,6,1,16,0);
				
//		SPI_OLED_Showdecimal(16,4,(RC_RX.KNOB_L+100)*10,6,1,16,0);
//		SPI_OLED_Showdecimal(16,6,(RC_RX.KNOB_R+100)*10,6,1,16,0);
		
		if(control_mode==1)
			SPI_OLED_ShowString(0,2,(char *)"     RC Mode    ",16,1);
		else 
			SPI_OLED_ShowString(0,2,(char *)"    AUTO Mode   ",16,1);
		osDelay(50);
	}
}
