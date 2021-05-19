/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "hw.h"
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
extern uint32_t g_vel_act; //actual motor velocity global variable from hw.c file
extern uint32_t g_vel_cmd; //global velocity command variable from hw.c file

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 96 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId tmc_taskHandle;
uint32_t uiTaskBuffer[ 96 ];
osStaticThreadDef_t uiTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void tmc_task_entry(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 96, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of tmc_task */
  osThreadStaticDef(tmc_task, tmc_task_entry, osPriorityHigh, 0, 96, uiTaskBuffer, &uiTaskControlBlock);
  tmc_taskHandle = osThreadCreate(osThread(tmc_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {


      if(hw_sw1()){
          tmc_startStepGen();
          tmc_commandVelocity(1600);
          tmc_direction(true);
      }
      else if(hw_sw3()){
          tmc_startStepGen();
          tmc_commandVelocity(1600);
          tmc_direction(false);
      }
      else{
          //tmc_stopStepGen();
          tmc_commandVelocity(0);
      }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_tmc_task_entry */
/**
* @brief Function implementing the tmc_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tmc_task_entry */
void tmc_task_entry(void const * argument)
{
  /* USER CODE BEGIN tmc_task_entry */
  /* Infinite loop */
  for(;;)
  {
      static uint32_t timing = 0;

      if(g_vel_cmd < g_vel_act){
          if(g_vel_act - g_vel_cmd <= TMC_VEL_CHNG_PER_MS*10){ //we can directly jump to commanded speed
              tmc_setVel(g_vel_cmd);
          }
          else{ //decrease speed max allowable amount
              tmc_setVel(g_vel_act - TMC_VEL_CHNG_PER_MS*10);
          }
      }
      else if(g_vel_cmd > g_vel_act){
          if(g_vel_cmd - g_vel_act <= TMC_VEL_CHNG_PER_MS*10){ //we can directly jump to commanded speed
              tmc_setVel(g_vel_cmd);
          }
          else{ //increase speed max allowable amount
              tmc_setVel(g_vel_act + TMC_VEL_CHNG_PER_MS*10);
          }
      }
      //else: nothing to do, speed is already mached
      osDelay(10);
  }
  /* USER CODE END tmc_task_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
