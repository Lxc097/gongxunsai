/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zdt_motor.h"
#include "dynamic_map_link.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct RC_DATA RC_RX;
uint8_t color_task[6]={0};

int circle_x[3]={880,1025,1160};  //红 绿 蓝
int circle_y[3]={1150,990,840};   //红 绿 蓝
struct VISUAL_DATA VIS_RX;  //视觉接收数据

int SpeedTarget[4]={0};

float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;

/* 先定义好存放数据的空间，队列大小是5，这里就定义5个。 */
UART2_RX_TypeDef uart2_rx_data_t[UART_BUFFER_QUANTITY];
UART5_RX_TypeDef uart5_rx_data_t[UART_BUFFER_QUANTITY];
UART1_RX_TypeDef uart1_rx_data_t[UART_BUFFER_QUANTITY];
/* 定义一个变量用作分配上面定义的数据 */
uint8_t uart2_buff_ctrl = 0;
uint8_t uart5_buff_ctrl = 0;
uint8_t uart1_buff_ctrl = 0;
/* 引用串口DMA handle  */
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* 队列 handle。去freertos.c文件里面找CubeMX给你创建好的。 */
extern osMessageQId uart2_queueHandle;
extern osMessageQId uart5_queueHandle;
extern osMessageQId uart1_queueHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Redirect printf function */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
    return (ch);
}
 
void USART2_DMAHandler(void)
{
    if (RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) // 判断是否是空闲中断
    {
        UART2_RX_TypeDef *pUart2Data; /* 定义指向创建串口数据的指针 */
 
        __HAL_UART_CLEAR_IDLEFLAG(&huart2); // 清除空闲中断标志
        HAL_UART_DMAStop(&huart2);          // 停止本次DMA传输
 
        /* 计算接收到的数据长度，放进串口数据结构体中 */
        uart2_rx_data_t[uart2_buff_ctrl].size = UART2_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
        /* 将这个串口数据的结构体地址给指针 */
        pUart2Data = &uart2_rx_data_t[uart2_buff_ctrl];
        /* 把指向串口接收数据的指针放入消息队列。注意，这里传的是指针的地址，也就是指针串口数据结构体的指针的地址不能传pUartData本身 */
        xQueueSendFromISR(uart2_queueHandle, &pUart2Data, NULL);
        /* 这里进行加一，使其下一次DMA传输时将接收的数据放到下一个串口缓冲区中 */
        uart2_buff_ctrl++;
        /* 取余操作防止越界 */
        uart2_buff_ctrl %= UART_BUFFER_QUANTITY;
        /* 重启开始DMA传输 */
        HAL_UART_Receive_DMA(&huart2, uart2_rx_data_t[uart2_buff_ctrl].buffer, UART2_BUFFER_SIZE);
    }
}

void USART5_DMAHandler(void)
{
    if (RESET != __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE)) // 判断是否是空闲中断
    {
        UART5_RX_TypeDef *pUart5Data; /* 定义指向创建串口数据的指针 */
 
        __HAL_UART_CLEAR_IDLEFLAG(&huart5); // 清除空闲中断标志
        HAL_UART_DMAStop(&huart5);          // 停止本次DMA传输
 
        /* 计算接收到的数据长度，放进串口数据结构体中 */
        uart5_rx_data_t[uart5_buff_ctrl].size = UART5_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
#if DYNAMIC_MAP_LINK_UART5
        DynamicMap_Link_ReceiveBytes(uart5_rx_data_t[uart5_buff_ctrl].buffer,
                                     uart5_rx_data_t[uart5_buff_ctrl].size);
#endif
        /* 将这个串口数据的结构体地址给指针 */
        pUart5Data = &uart5_rx_data_t[uart5_buff_ctrl];
        /* 把指向串口接收数据的指针放入消息队列。注意，这里传的是指针的地址，也就是指针串口数据结构体的指针的地址不能传pUartData本身 */
        xQueueSendFromISR(uart5_queueHandle, &pUart5Data, NULL);
        /* 这里进行加一，使其下一次DMA传输时将接收的数据放到下一个串口缓冲区中 */
        uart5_buff_ctrl++;
        /* 取余操作防止越界 */
        uart5_buff_ctrl %= UART_BUFFER_QUANTITY;
        /* 重启开始DMA传输 */
        HAL_UART_Receive_DMA(&huart5, uart5_rx_data_t[uart5_buff_ctrl].buffer, UART5_BUFFER_SIZE);
    }
}

void USART1_DMAHandler(void)
{
    if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) // 判断是否是空闲中断
    {
        UART1_RX_TypeDef *pUart1Data; /* 定义指向创建串口数据的指针 */
 
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); // 清除空闲中断标志
        HAL_UART_DMAStop(&huart1);          // 停止本次DMA传输
 
        /* 计算接收到的数据长度，放进串口数据结构体中 */
        uart1_rx_data_t[uart1_buff_ctrl].size = UART1_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        /* 将这个串口数据的结构体地址给指针 */
        pUart1Data = &uart1_rx_data_t[uart1_buff_ctrl];
        /* 把指向串口接收数据的指针放入消息队列。注意，这里传的是指针的地址，也就是指针串口数据结构体的指针的地址不能传pUartData本身 */
        xQueueSendFromISR(uart1_queueHandle, &pUart1Data, NULL);
        /* 这里进行加一，使其下一次DMA传输时将接收的数据放到下一个串口缓冲区中 */
        uart1_buff_ctrl++;
        /* 取余操作防止越界 */
        uart1_buff_ctrl %= UART_BUFFER_QUANTITY;
        /* 重启开始DMA传输 */
        HAL_UART_Receive_DMA(&huart1, uart1_rx_data_t[uart1_buff_ctrl].buffer, UART1_BUFFER_SIZE);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);    //使能串口中断
  HAL_UART_Receive_DMA(&huart2, uart2_rx_data_t[uart2_buff_ctrl].buffer, UART2_BUFFER_SIZE);    //使能串口DMA
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart5, uart5_rx_data_t[uart5_buff_ctrl].buffer, UART5_BUFFER_SIZE);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);    //使能串口中断
  HAL_UART_Receive_DMA(&huart1, uart1_rx_data_t[uart1_buff_ctrl].buffer, UART1_BUFFER_SIZE);    //使能串口DMA
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
/* The final vehicle requires its 8 MHz HSE.  A separate static bring-up
 * target may use HSI so the rest of a newly assembled board can be checked
 * while an HSE soldering/oscillation fault is repaired. */
#if defined(VEHICLE_STATIC_HSI_CLOCK) && VEHICLE_STATIC_HSI_CLOCK
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#endif
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
#if defined(VEHICLE_STATIC_HSI_CLOCK) && VEHICLE_STATIC_HSI_CLOCK
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
#else
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
#endif
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* CAN1 接收中断 */
    if(hcan->Instance == CAN1)
    {
        CAN_RxHeaderTypeDef   Rx_Header;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&Rx_Header, data) == HAL_OK)
        {
            if(Rx_Header.RTR != CAN_RTR_DATA) return;
            if(Rx_Header.IDE == CAN_ID_EXT) {
                ZdtMotor_OnCanFrame(Rx_Header.ExtId, (uint8_t)Rx_Header.DLC, data);
                /* Do not pass wheel extended frames to the legacy handler:
                 * its StdId==0 fallback belongs only to standard G6220 data. */
                if(Rx_Header.ExtId == 0x300U || Rx_Header.ExtId == 0x400U)
                    Response_Handle(Rx_Header.ExtId, 0xFFFFFFFFU, data);
            } else {
                Response_Handle(0U, Rx_Header.StdId, data);
            }
        }    
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
