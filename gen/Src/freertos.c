/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
osThreadId DistSensorTaskHandle;
uint32_t DistSensorTaskBuffer[ 1024 ];
osStaticThreadDef_t DistSensorTaskControlBlock;
osThreadId DebugTaskHandle;
uint32_t DebugTaskBuffer[ 1024 ];
osStaticThreadDef_t DebugTaskControlBlock;
osThreadId ControlTaskHandle;
uint32_t ControlTaskBuffer[ 1024 ];
osStaticThreadDef_t ControlTaskControlBlock;
osThreadId ProgLabyrinthTaHandle;
uint32_t ProgLabyrinthTaskBuffer[ 1024 ];
osStaticThreadDef_t ProgLabyrinthTaskControlBlock;
osThreadId ProgRaceTrackTaHandle;
uint32_t ProgRaceTrackTaskBuffer[ 1024 ];
osStaticThreadDef_t ProgRaceTrackTaskControlBlock;
osThreadId GyroTaskHandle;
uint32_t GyroTaskBuffer[ 256 ];
osStaticThreadDef_t GyroTaskControlBlock;
osThreadId LineDetectTaskHandle;
uint32_t LineDetectTaskBuffer[ 1024 ];
osStaticThreadDef_t LineDetectTaskControlBlock;
osThreadId StartupTaskHandle;
uint32_t StartupTaskBuffer[ 256 ];
osStaticThreadDef_t StartupTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void runDistSensorTask(const void *argument);
void runGyroTask(const void *argument);
void runLineDetectTask(const void *argument);
void runDebugTask(const void *argument);
void runControlTask(const void *argument);
void runProgLabyrinthTask(const void *argument);
void runProgRaceTrackTask(const void *argument);
void runStartupTask(const void *argument);
/* USER CODE END FunctionPrototypes */

void StartDistSensorTask(void const * argument);
void StartDebugTask(void const * argument);
void StartControlTask(void const * argument);
void StartProgLabyrinthTask(void const * argument);
void StartProgRaceTrackTask(void const * argument);
void StartGyroTask(void const * argument);
void StartLineDetectTask(void const * argument);
void StartStartupTask(void const * argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DistSensorTask */
  osThreadStaticDef(DistSensorTask, StartDistSensorTask, osPriorityNormal, 0, 1024, DistSensorTaskBuffer, &DistSensorTaskControlBlock);
  DistSensorTaskHandle = osThreadCreate(osThread(DistSensorTask), NULL);

  /* definition and creation of DebugTask */
  osThreadStaticDef(DebugTask, StartDebugTask, osPriorityLow, 0, 1024, DebugTaskBuffer, &DebugTaskControlBlock);
  DebugTaskHandle = osThreadCreate(osThread(DebugTask), NULL);

  /* definition and creation of ControlTask */
  osThreadStaticDef(ControlTask, StartControlTask, osPriorityRealtime, 0, 1024, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of ProgLabyrinthTa */
  osThreadStaticDef(ProgLabyrinthTa, StartProgLabyrinthTask, osPriorityNormal, 0, 1024, ProgLabyrinthTaskBuffer, &ProgLabyrinthTaskControlBlock);
  ProgLabyrinthTaHandle = osThreadCreate(osThread(ProgLabyrinthTa), NULL);

  /* definition and creation of ProgRaceTrackTa */
  osThreadStaticDef(ProgRaceTrackTa, StartProgRaceTrackTask, osPriorityNormal, 0, 1024, ProgRaceTrackTaskBuffer, &ProgRaceTrackTaskControlBlock);
  ProgRaceTrackTaHandle = osThreadCreate(osThread(ProgRaceTrackTa), NULL);

  /* definition and creation of GyroTask */
  osThreadStaticDef(GyroTask, StartGyroTask, osPriorityNormal, 0, 256, GyroTaskBuffer, &GyroTaskControlBlock);
  GyroTaskHandle = osThreadCreate(osThread(GyroTask), NULL);

  /* definition and creation of LineDetectTask */
  osThreadStaticDef(LineDetectTask, StartLineDetectTask, osPriorityHigh, 0, 1024, LineDetectTaskBuffer, &LineDetectTaskControlBlock);
  LineDetectTaskHandle = osThreadCreate(osThread(LineDetectTask), NULL);

  /* definition and creation of StartupTask */
  osThreadStaticDef(StartupTask, StartStartupTask, osPriorityBelowNormal, 0, 256, StartupTaskBuffer, &StartupTaskControlBlock);
  StartupTaskHandle = osThreadCreate(osThread(StartupTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDistSensorTask */
/**
  * @brief  Function implementing the DistSensorTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDistSensorTask */
void StartDistSensorTask(void const * argument)
{
    
    
    
    
    
    
    
    
    

  /* USER CODE BEGIN StartDistSensorTask */
  runDistSensorTask(argument);
  /* USER CODE END StartDistSensorTask */
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

/* USER CODE BEGIN Header_StartProgLabyrinthTask */
/**
* @brief Function implementing the ProgLabyrinthTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProgLabyrinthTask */
void StartProgLabyrinthTask(void const * argument)
{
  /* USER CODE BEGIN StartProgLabyrinthTask */
  runProgLabyrinthTask(argument);
  /* USER CODE END StartProgLabyrinthTask */
}

/* USER CODE BEGIN Header_StartProgRaceTrackTask */
/**
* @brief Function implementing the ProgRaceTrackTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProgRaceTrackTask */
void StartProgRaceTrackTask(void const * argument)
{
  /* USER CODE BEGIN StartProgRaceTrackTask */
  runProgRaceTrackTask(argument);
  /* USER CODE END StartProgRaceTrackTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
* @brief Function implementing the GyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void const * argument)
{
  /* USER CODE BEGIN StartGyroTask */
  runGyroTask(argument);
  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartLineDetectTask */
/**
* @brief Function implementing the LineDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLineDetectTask */
void StartLineDetectTask(void const * argument)
{
  /* USER CODE BEGIN StartLineDetectTask */
  runLineDetectTask(argument);
  /* USER CODE END StartLineDetectTask */
}

/* USER CODE BEGIN Header_StartStartupTask */
/**
* @brief Function implementing the StartupTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStartupTask */
void StartStartupTask(void const * argument)
{
  /* USER CODE BEGIN StartStartupTask */
  runStartupTask(argument);
  /* USER CODE END StartStartupTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
