/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "io_devices.h"
#include "OLED_SPI.h"
#include "OLED_SoftSPI.h"
#include "position.h"
#include "Remote_Control_RX.h"
#include "chassis_control.h"
#include "tower.h"
#include "visual_identity.h"

/* 一、引用标准输入输出库 */
#include <stdio.h>
#include "FreeRTOS.h"  
#include "queue.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* 二、定义串口数据缓冲区配置 */
#define UART2_BUFFER_SIZE        (32)    //串口2缓冲区字节
#define UART5_BUFFER_SIZE        (16)    //串口5缓冲区字节
#define UART1_BUFFER_SIZE        (16)    //串口1缓冲区字节
#define UART6_BUFFER_SIZE        (16)    //串口6动态地图缓冲区字节
#define UART_BUFFER_QUANTITY     (5)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* 三、定义串口数据结构体，DMA传输的数据将保存在这里 */
typedef struct
{
    uint8_t buffer[UART2_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART2_RX_TypeDef;

typedef struct
{
    uint8_t buffer[UART5_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART5_RX_TypeDef;

typedef struct
{
    uint8_t buffer[UART1_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART1_RX_TypeDef;

typedef struct
{
    uint8_t buffer[UART6_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART6_RX_TypeDef;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_RST_Pin GPIO_PIN_2
#define SPI2_RST_GPIO_Port GPIOE
#define SPI2_DC_Pin GPIO_PIN_3
#define SPI2_DC_GPIO_Port GPIOE
#define SPI2_CS_Pin GPIO_PIN_4
#define SPI2_CS_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOC
#define SPI2_MOSI_Pin GPIO_PIN_3
#define SPI2_MOSI_GPIO_Port GPIOC
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define VM_SW_Pin GPIO_PIN_0
#define VM_SW_GPIO_Port GPIOD
#define Cam_Light_Pin GPIO_PIN_1
#define Cam_Light_GPIO_Port GPIOD
#define Start_Pin GPIO_PIN_3
#define Start_GPIO_Port GPIOD
#define SPI1_DC_Pin GPIO_PIN_4
#define SPI1_DC_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_5
#define SPI1_CS_GPIO_Port GPIOD
#define SPI1_RST_Pin GPIO_PIN_6
#define SPI1_RST_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
