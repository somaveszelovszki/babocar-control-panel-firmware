/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <uns/util/types.h>
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
osThreadId IdleTaskHandle;
uint32_t IdleTaskBuffer[ 128 ];
osStaticThreadDef_t IdleTaskControlBlock;
osThreadId ControlTaskHandle;
uint32_t ControlTaskBuffer[ 256 ];
osStaticThreadDef_t ControlTaskControlBlock;
osThreadId ProgTrackTaskHandle;
uint32_t ProgTrackTaskBuffer[ 512 ];
osStaticThreadDef_t ProgTrackTaskControlBlock;
osThreadId DebugTaskHandle;
uint32_t DebugTaskBuffer[ 1024 ];
osStaticThreadDef_t DebugTaskControlBlock;
osThreadId RadioStartTaskHandle;
uint32_t RadioStartTaskBuffer[ 128 ];
osStaticThreadDef_t RadioStartTaskControlBlock;
osMessageQId LogQueueHandle;
uint8_t LogQueueBuffer[ 16 * sizeof( LogQueueItem_t ) ];
osStaticMessageQDef_t LogQueueControlBlock;
osMessageQId ControlPropsQueueHandle;
uint8_t ControlPropsQueueBuffer[ 1 * sizeof( ControlPropsQueueItem_t ) ];
osStaticMessageQDef_t ControlPropsQueueControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void runDebugTask(void const *argument);
void runControlTask(void const *argument);
void runProgTrackTask(void const *argument);
void runRadioStartTask(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartIdleTask(const void * argument);
void StartControlTask(const void * argument);
void StartProgTrackTask(const void * argument);
void StartDebugTask(const void * argument);
void StartRadioStartTask(const void * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* Create the thread(s) */
  /* definition and creation of IdleTask */
  osThreadStaticDef(IdleTask, StartIdleTask, osPriorityIdle, 0, 128, IdleTaskBuffer, &IdleTaskControlBlock);
  IdleTaskHandle = osThreadCreate(osThread(IdleTask), NULL);

  /* definition and creation of ControlTask */
  osThreadStaticDef(ControlTask, StartControlTask, osPriorityRealtime, 0, 256, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of ProgTrackTask */
  osThreadStaticDef(ProgTrackTask, StartProgTrackTask, osPriorityNormal, 0, 512, ProgTrackTaskBuffer, &ProgTrackTaskControlBlock);
  ProgTrackTaskHandle = osThreadCreate(osThread(ProgTrackTask), NULL);

  /* definition and creation of DebugTask */
  osThreadStaticDef(DebugTask, StartDebugTask, osPriorityLow, 0, 1024, DebugTaskBuffer, &DebugTaskControlBlock);
  DebugTaskHandle = osThreadCreate(osThread(DebugTask), NULL);

  /* definition and creation of RadioStartTask */
  osThreadStaticDef(RadioStartTask, StartRadioStartTask, osPriorityNormal, 0, 128, RadioStartTaskBuffer, &RadioStartTaskControlBlock);
  RadioStartTaskHandle = osThreadCreate(osThread(RadioStartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of LogQueue */
  osMessageQStaticDef(LogQueue, 16, LogQueueItem_t, LogQueueBuffer, &LogQueueControlBlock);
  LogQueueHandle = osMessageCreate(osMessageQ(LogQueue), NULL);

  /* definition and creation of ControlPropsQueue */
  osMessageQStaticDef(ControlPropsQueue, 1, ControlPropsQueueItem_t, ControlPropsQueueBuffer, &ControlPropsQueueControlBlock);
  ControlPropsQueueHandle = osMessageCreate(osMessageQ(ControlPropsQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartIdleTask */
/**
  * @brief  Function implementing the IdleTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartIdleTask */
void StartIdleTask(void const * argument)
{

  /* USER CODE BEGIN StartIdleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartIdleTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  runControlTask(argument);
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartProgTrackTask */
/**
* @brief Function implementing the ProgTrackTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProgTrackTask */
void StartProgTrackTask(void const * argument)
{
  /* USER CODE BEGIN StartProgTrackTask */
  runProgTrackTask(argument);
  /* USER CODE END StartProgTrackTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void const * argument)
{
  /* USER CODE BEGIN StartDebugTask */
  runDebugTask(argument);
  /* USER CODE END StartDebugTask */
}

/* USER CODE BEGIN Header_StartRadioStartTask */
/**
* @brief Function implementing the RadioStartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadioStartTask */
void StartRadioStartTask(void const * argument)
{
  /* USER CODE BEGIN StartRadioStartTask */
  runRadioStartTask(argument);
  /* USER CODE END StartRadioStartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
