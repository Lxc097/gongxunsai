/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Main_task.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CHASSIS_TASK */
osThreadId_t CHASSIS_TASKHandle;
const osThreadAttr_t CHASSIS_TASK_attributes = {
  .name = "CHASSIS_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OUTPU_TASK */
osThreadId_t OUTPU_TASKHandle;
const osThreadAttr_t OUTPU_TASK_attributes = {
  .name = "OUTPU_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_RX_TASK */
osThreadId_t UART_RX_TASKHandle;
const osThreadAttr_t UART_RX_TASK_attributes = {
  .name = "UART_RX_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MAIN_TASK */
osThreadId_t MAIN_TASKHandle;
const osThreadAttr_t MAIN_TASK_attributes = {
  .name = "MAIN_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TOWER_TASK */
osThreadId_t TOWER_TASKHandle;
const osThreadAttr_t TOWER_TASK_attributes = {
  .name = "TOWER_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for uart2_queue */
osMessageQueueId_t uart2_queueHandle;
const osMessageQueueAttr_t uart2_queue_attributes = {
  .name = "uart2_queue"
};
/* Definitions for uart5_queue */
osMessageQueueId_t uart5_queueHandle;
const osMessageQueueAttr_t uart5_queue_attributes = {
  .name = "uart5_queue"
};
/* Definitions for uart1_queue */
osMessageQueueId_t uart1_queueHandle;
const osMessageQueueAttr_t uart1_queue_attributes = {
  .name = "uart1_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Chassis_Task(void *argument);
void Output_Task(void *argument);
void UART_RX_Task(void *argument);
void Main_Task(void *argument);
void Tower_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart2_queue */
  uart2_queueHandle = osMessageQueueNew (4, sizeof(void *), &uart2_queue_attributes);

  /* creation of uart5_queue */
  uart5_queueHandle = osMessageQueueNew (4, sizeof(void *), &uart5_queue_attributes);

  /* creation of uart1_queue */
  uart1_queueHandle = osMessageQueueNew (4, sizeof(void *), &uart1_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CHASSIS_TASK */
  CHASSIS_TASKHandle = osThreadNew(Chassis_Task, NULL, &CHASSIS_TASK_attributes);

  /* creation of OUTPU_TASK */
  OUTPU_TASKHandle = osThreadNew(Output_Task, NULL, &OUTPU_TASK_attributes);

  /* creation of UART_RX_TASK */
  UART_RX_TASKHandle = osThreadNew(UART_RX_Task, NULL, &UART_RX_TASK_attributes);

  /* creation of MAIN_TASK */
  MAIN_TASKHandle = osThreadNew(Main_Task, NULL, &MAIN_TASK_attributes);

  /* creation of TOWER_TASK */
  TOWER_TASKHandle = osThreadNew(Tower_Task, NULL, &TOWER_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the CHASSIS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    Chassis_task();
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Output_Task */
/**
* @brief Function implementing the OUTPU_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Output_Task */
void Output_Task(void *argument)
{
  /* USER CODE BEGIN Output_Task */
  /* Infinite loop */
  for(;;)
  {
    Output_task();
  }
  /* USER CODE END Output_Task */
}

/* USER CODE BEGIN Header_UART_RX_Task */
/**
* @brief Function implementing the UART_RX_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_RX_Task */
void UART_RX_Task(void *argument)
{
  /* USER CODE BEGIN UART_RX_Task */
	
	//UART1 -- NANO
	//UART2 -- OPS
	//UART4 -- Chassis
	//UART5 -- RC
	//UART6 -- Jetson动态地图（H27扩展口，替代未启用的ESP07预留）
	
	UART2_RX_TypeDef *pRecvUart2Data; /* 定义指向串口数据的指针 */
	UART5_RX_TypeDef *pRecvUart5Data; /* 定义指向串口数据的指针 */
	UART1_RX_TypeDef *pRecvUart1Data; /* 定义指向串口数据的指针 */
	printf("Uart Thread init. \r\n");
	osDelay(500);
//	System_message("Uart RX init");
	
  /* Infinite loop */
  for(;;)
  {
    /* 一直等待直到有数据 */
    /* 参数1：队列handle */
    /* 参数2：放入接收消息的指针，也就是让它指向串口中断中已接收串口数据完成的结构体 */
    /* 参数3：等待时间，这里是等待永久，直到有数据 */
    if (xQueueReceive(uart2_queueHandle, &pRecvUart2Data, 0) == pdTRUE)
    {
        /* 打印或处理接收到的数据 */
			  POS_Data_Unpack(pRecvUart2Data->buffer);
    }
		if (xQueueReceive(uart5_queueHandle, &pRecvUart5Data, 0) == pdTRUE)
    {
        /* 打印或处理接收到的数据 */
		  	RC_Data_Unpack(pRecvUart5Data->buffer);
    }
		if (xQueueReceive(uart1_queueHandle, &pRecvUart1Data, 0) == pdTRUE)
    {
        /* 打印或处理接收到的数据 */
        NANO_Data_Unpack(pRecvUart1Data->buffer);     //NANO接收解包
    }
		osDelay(5);
  }
  /* USER CODE END UART_RX_Task */
}

/* USER CODE BEGIN Header_Main_Task */
/**
* @brief Function implementing the MAIN_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Main_Task */
void Main_Task(void *argument)
{
  /* USER CODE BEGIN Main_Task */
  /* Infinite loop */
  for(;;)
  {
    Main_task();
  }
  /* USER CODE END Main_Task */
}

/* USER CODE BEGIN Header_Tower_Task */
/**
* @brief Function implementing the TOWER_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tower_Task */
void Tower_Task(void *argument)
{
  /* USER CODE BEGIN Tower_Task */
  /* Infinite loop */
  for(;;)
  {
    Tower_task();
  }
  /* USER CODE END Tower_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
