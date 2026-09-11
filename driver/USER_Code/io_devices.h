#ifndef __IO_DEVICES__
#define __IO_DEVICES__

#include "main.h"
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

 //LED_1  NANO已就位
 //LED_2  底盘视觉矫正
 //LED_3  塔吊运行

#define LED_1_ON()         HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_RESET)
#define LED_1_OFF()        HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,GPIO_PIN_SET)
#define LED_1_TOG()        HAL_GPIO_TogglePin(LED_1_GPIO_Port,LED_1_Pin)

#define LED_2_ON()         HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_RESET)
#define LED_2_OFF()        HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin,GPIO_PIN_SET)
#define LED_2_TOG()        HAL_GPIO_TogglePin(LED_2_GPIO_Port,LED_2_Pin)

#define LED_3_ON()         HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_RESET)
#define LED_3_OFF()        HAL_GPIO_WritePin(LED_3_GPIO_Port,LED_3_Pin,GPIO_PIN_SET)
#define LED_3_TOG()        HAL_GPIO_TogglePin(LED_3_GPIO_Port,LED_3_Pin)

#define CAM_Light_ON()         HAL_GPIO_WritePin(Cam_Light_GPIO_Port,Cam_Light_Pin,GPIO_PIN_SET)
#define CAM_Light_OFF()        HAL_GPIO_WritePin(Cam_Light_GPIO_Port,Cam_Light_Pin,GPIO_PIN_RESET)

#define Buzzer_ON()         HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET)
#define Buzzer_OFF()        HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET)

#define VM_ON()         HAL_GPIO_WritePin(VM_SW_GPIO_Port,VM_SW_Pin,GPIO_PIN_SET)
#define VM_OFF()        HAL_GPIO_WritePin(VM_SW_GPIO_Port,VM_SW_Pin,GPIO_PIN_RESET)

#define Cam_Light_ON()         HAL_GPIO_WritePin(Cam_Light_GPIO_Port,Cam_Light_Pin,GPIO_PIN_SET)
#define Cam_Light_OFF()        HAL_GPIO_WritePin(Cam_Light_GPIO_Port,Cam_Light_Pin,GPIO_PIN_RESET)

void LED_1_flashing(uint16_t delay_on, uint16_t delay_off, uint8_t times);
void LED_2_flashing(uint16_t delay_on, uint16_t delay_off, uint8_t times);
void LED_3_flashing(uint16_t delay_on, uint16_t delay_off, uint8_t times);
void buzzer_rings(uint16_t delay_on, uint16_t delay_off, uint8_t times);

#endif  /* __IO_DEVICES__ */
