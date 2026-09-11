#include "io_devices.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 LED1闪烁
//  @return     		 /
//  @param					 uint16_t delay_on    点亮时间
//  @param					 uint16_t delay_off   熄灭时间
//  @param					 uint8_t times        次数
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void LED_1_flashing(uint16_t delay_on, uint16_t delay_off, uint8_t times)
{
	for(uint8_t i=times; i>0; i--)
	{
		LED_1_ON();
		osDelay(delay_on);
		LED_1_OFF();
		if (i>1)
			osDelay(delay_off);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 LED2闪烁
//  @return     		 /
//  @param					 uint16_t delay_on    点亮时间
//  @param					 uint16_t delay_off   熄灭时间
//  @param					 uint8_t times        次数
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void LED_2_flashing(uint16_t delay_on, uint16_t delay_off, uint8_t times)
{
	for(uint8_t i=times; i>0; i--)
	{
		LED_2_ON();
		osDelay(delay_on);
		LED_2_OFF();
		if (i>1)
			osDelay(delay_off);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 LED3闪烁
//  @return     		 /
//  @param					 uint16_t delay_on    点亮时间
//  @param					 uint16_t delay_off   熄灭时间
//  @param					 uint8_t times        次数
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void LED_3_flashing(uint16_t delay_on, uint16_t delay_off, uint8_t times)
{
	for(uint8_t i=times; i>0; i--)
	{
		LED_3_ON();
		osDelay(delay_on);
		LED_3_OFF();
		if (i>1)
			osDelay(delay_off);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 蜂鸣器
//  @return     		 /
//  @param					 uint16_t delay_on    发声时间
//  @param					 uint16_t delay_off   沉默时间
//  @param					 uint8_t times        次数
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void buzzer_rings(uint16_t delay_on, uint16_t delay_off, uint8_t times)
{
	for(uint8_t i=times; i>0; i--)
	{
		Buzzer_ON();
		osDelay(delay_on);
		Buzzer_OFF();
		if (i>1)
			osDelay(delay_off);
    }
}
