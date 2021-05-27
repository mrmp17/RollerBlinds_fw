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
extern int32_t g_vel_act; //actual motor velocity global variable from hw.c file
extern int32_t g_vel_cmd; //global velocity command variable from hw.c file

extern int32_t g_steps_abs; //global variables for absolute step counter and stepper direction from hw.c file

extern int32_t g_pos_cmd; //global position command variable from hw.c file
extern bool g_pos_ctrl_active; //global positional controll active flag from hw.c file

extern bool g_charging_flag; //global charging flag from hw.c file
extern bool g_low_battery_flag; //global low batterz flag from hw.c
extern bool g_balance_active_flag; //global balance active flag from hw.c
extern bool g_fully_charged_flag; //global fullu charged flag from hw.c

/* USER CODE END Variables */
osThreadId misc_taskHandle;
uint32_t defaultTaskBuffer[ 96 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId tmc_taskHandle;
uint32_t uiTaskBuffer[ 96 ];
osStaticThreadDef_t uiTaskControlBlock;
osThreadId main_logic_taskHandle;
uint32_t main_logic_taskBuffer[ 96 ];
osStaticThreadDef_t main_logic_taskControlBlock;
osThreadId esp_taskHandle;
uint32_t esp_taskBuffer[ 96 ];
osStaticThreadDef_t esp_taskControlBlock;
osThreadId analog_taskHandle;
uint32_t analog_taskBuffer[ 96 ];
osStaticThreadDef_t analog_taskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void misc_task_entry(void const * argument);
void tmc_task_entry(void const * argument);
void main_logic_task_entry(void const * argument);
void esp_task_entry(void const * argument);
void analog_task_entry(void const * argument);

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
  /* definition and creation of misc_task */
  osThreadStaticDef(misc_task, misc_task_entry, osPriorityNormal, 0, 96, defaultTaskBuffer, &defaultTaskControlBlock);
  misc_taskHandle = osThreadCreate(osThread(misc_task), NULL);

  /* definition and creation of tmc_task */
  osThreadStaticDef(tmc_task, tmc_task_entry, osPriorityHigh, 0, 96, uiTaskBuffer, &uiTaskControlBlock);
  tmc_taskHandle = osThreadCreate(osThread(tmc_task), NULL);

  /* definition and creation of main_logic_task */
  osThreadStaticDef(main_logic_task, main_logic_task_entry, osPriorityNormal, 0, 96, main_logic_taskBuffer, &main_logic_taskControlBlock);
  main_logic_taskHandle = osThreadCreate(osThread(main_logic_task), NULL);

  /* definition and creation of esp_task */
  osThreadStaticDef(esp_task, esp_task_entry, osPriorityNormal, 0, 96, esp_taskBuffer, &esp_taskControlBlock);
  esp_taskHandle = osThreadCreate(osThread(esp_task), NULL);

  /* definition and creation of analog_task */
  osThreadStaticDef(analog_task, analog_task_entry, osPriorityNormal, 0, 96, analog_taskBuffer, &analog_taskControlBlock);
  analog_taskHandle = osThreadCreate(osThread(analog_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_misc_task_entry */
/**
  * @brief  Function implementing the misc_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_misc_task_entry */
void misc_task_entry(void const * argument)
{
  /* USER CODE BEGIN misc_task_entry */
  /* Infinite loop */
    for(;;)
    {


        if(hw_sw1()){
            //tmc_startStepGen();
            //tmc_commandVelocity(5000);
            //tmc_direction(true);

            //hw_tmcEnable(true);
            //tmc_startStepGen();
            hw_redLed(true);
            tmc_commandPosition(g_steps_abs+205000);
            while(tmc_posCtrlActive());
            //tmc_commandPosition(g_steps_abs+10000);
            //while(tmc_posCtrlActive());
            //tmc_commandVelocity(-4000);

            //tmc_stopStepGen();
            //hw_tmcEnable(false);
        }
        else if(hw_sw2()){
            //tmc_commandVelocity(4000);
        }
        else if(hw_sw3()){
            //tmc_startStepGen();
            //tmc_commandVelocity(-5000);
            //tmc_direction(false);

            //hw_tmcEnable(true);
            //tmc_startStepGen();

            //tmc_commandPosition(g_steps_abs-205000);
            hw_redLed(true);
            tmc_commandPosition(g_steps_abs-205000);
            while(tmc_posCtrlActive());
            //tmc_stopStepGen();
            //hw_tmcEnable(false);
        }
        else{
            //tmc_stopStepGen();
            tmc_commandVelocity(0);
        }
        if(g_charging_flag){
            hw_blueLed(true);
        }
        else{
            hw_sleep();
            hw_blueLed(false);
        }


    }
  /* USER CODE END misc_task_entry */
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
  //while(1);
  for(;;)
  {
      //positional controll. sets desired velocity
      //no need to ramp speed up, is handeled by velocity ctrl. only need to slow down to reach the exact destination

      if(g_pos_ctrl_active){ //positional controll active / position not yet reached
          int32_t diff = g_pos_cmd - g_steps_abs;
//          else if((g_pos_cmd >= 0 && g_steps_abs > g_pos_cmd) || (g_pos_cmd < 0 && g_steps_abs < g_pos_cmd)){
//              //we have overshot destination in this case. whoops.
//              g_pos_ctrl_active = false;
//          }
          if(abs(diff) < TMC_POS_CTRL_RAMP_DOWN_LEN){ //near destinatio
              int32_t reqVel = (TMC_CRUISE_VEL*diff)/TMC_POS_CTRL_RAMP_DOWN_LEN; //required velocity. this should be capped at TMC_MIN_VEL (min velocity at which destination is reached)
              if(reqVel >= 0 && reqVel <= TMC_MIN_VEL) reqVel = TMC_MIN_VEL;
              else if(reqVel < 0 && reqVel >= -TMC_MIN_VEL) reqVel = -TMC_MIN_VEL;
              tmc_commandVelocity(reqVel);
          }
          else if(diff > 0){ //cruising speed, positive direction
              tmc_commandVelocity(TMC_CRUISE_VEL);
          }
          else{ //cruising speed, negative direction
              tmc_commandVelocity(-TMC_CRUISE_VEL);
          }
          if(abs(diff) <= 5){ //5 steps to destination is close enough
              g_pos_ctrl_active = false;
              tmc_commandVelocity(0);
          }
      }

      //velocity controll
      int32_t sps_rel;
      if(g_vel_cmd != 0 || g_vel_act != 0){ //commanded and actual velocity not zero, tmc should be ON
          tmc_startStepGen();
          hw_tmcEnable(true);
          if(g_vel_cmd < g_vel_act){
              if(g_vel_act - g_vel_cmd <= TMC_VEL_CHNG_PER_MS*20){ //we can directly jump to commanded speed
                  sps_rel = g_vel_cmd;
                  g_vel_act = sps_rel;
                  if(sps_rel >= 0){
                      tmc_setSpS(sps_rel);
                      tmc_direction(true);
                  }
                  else{
                      tmc_setSpS(-sps_rel);
                      tmc_direction(false);
                  }

              }
              else{ //decrease speed max allowable amount
                  sps_rel = g_vel_act - TMC_VEL_CHNG_PER_MS * 20;
                  g_vel_act = sps_rel;
                  if(sps_rel >= 0){
                      tmc_setSpS(sps_rel);
                      tmc_direction(true);
                  }
                  else{
                      tmc_setSpS(-sps_rel);
                      tmc_direction(false);
                  }
              }
          }
          else if(g_vel_cmd > g_vel_act){
              if(g_vel_cmd - g_vel_act <= TMC_VEL_CHNG_PER_MS*20){ //we can directly jump to commanded speed
                  sps_rel = g_vel_cmd;
                  g_vel_act = sps_rel;
                  if(sps_rel >= 0){
                      tmc_setSpS(sps_rel);
                      tmc_direction(true);
                  }
                  else{
                      tmc_setSpS(-sps_rel);
                      tmc_direction(false);
                  }
              }
              else{ //increase speed max allowable amount
                  sps_rel = g_vel_act + TMC_VEL_CHNG_PER_MS * 20;
                  g_vel_act = sps_rel;
                  if(sps_rel >= 0){
                      tmc_setSpS(sps_rel);
                      tmc_direction(true);
                  }
                  else{
                      tmc_setSpS(-sps_rel);
                      tmc_direction(false);
                  }
              }
          }
      }
      else{ //commanded and actual velocity zero, tmc should be OFF
          tmc_stopStepGen();
          hw_tmcEnable(false);
      }

      //else: nothing to do, speed is already mached
      osDelay(20);
  }
  /* USER CODE END tmc_task_entry */
}

/* USER CODE BEGIN Header_main_logic_task_entry */
/**
* @brief Function implementing the main_logic_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_main_logic_task_entry */
void main_logic_task_entry(void const * argument)
{
  /* USER CODE BEGIN main_logic_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    uint32_t v1 = hw_getCell1Voltage();
    uint32_t v2 = hw_getCell2Voltage();
    uint32_t v3 = hw_getPackVoltage();
    dump(v1);
    dump(v2);
    dump(v3);

//    dbg_debugPrint("time\n");
//    uint8_t cajt[6] = {0};
//
//    cajt[0] = hw_getDay();
//    cajt[1] = hw_getMonth();
//    cajt[2] = hw_getYear();
//
//    cajt[3] = hw_getHour();
//    cajt[4] = hw_getMinute();
//    cajt[5] = hw_getSecond();
//
//    uint32_t strom = hw_getCell1Voltage();
//    dump(strom);
//    dump(cajt[1]);
//
//    dbg_debugPrint("slp\n");
//    osDelay(100);
//    //hw_sleep();
//    hw_redLed(true);
//    dbg_debugPrint("wkp\n");
  }
  /* USER CODE END main_logic_task_entry */
}

/* USER CODE BEGIN Header_esp_task_entry */
/**
* @brief Function implementing the esp_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_esp_task_entry */
void esp_task_entry(void const * argument)
{
  /* USER CODE BEGIN esp_task_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    uint32_t sdf = hw_getVbusVoltage();
    dump(sdf);
  }
  /* USER CODE END esp_task_entry */
}

/* USER CODE BEGIN Header_analog_task_entry */
/**
* @brief Function implementing the analog_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_analog_task_entry */
void analog_task_entry(void const * argument)
{
  /* USER CODE BEGIN analog_task_entry */
  /* Infinite loop */
  for(;;)
  {
      //low battery flag logic (whit hysteresis)
      if(!g_low_battery_flag && (hw_getCell1Voltage()<LOW_CELL_VOLT_THR || hw_getCell2Voltage()<LOW_CELL_VOLT_THR)){
          //if low battery flag not set and one cell dips below LOW_CELL_VOLT_THR, set low bat flag
          g_low_battery_flag = true;
      }
      if(g_low_battery_flag && (hw_getCell1Voltage()>LOW_CELL_VOLT_REC || hw_getCell2Voltage()>LOW_CELL_VOLT_REC)){
          //if low battery flag is set and both cells rise above LOW_CELL_VOLT_REC, reset low bat flag
          g_low_battery_flag = false;
      }

      //charging flag logic
      if(hw_vbusPresent()){
          g_charging_flag = true;
          if(hw_getCell1Voltage() > BAT_FULL_CHRG_THR && hw_getCell2Voltage() > BAT_FULL_CHRG_THR){
              //battery is fully charged
              g_fully_charged_flag = true;
          }
      }
      else{
          g_charging_flag = false;
          g_fully_charged_flag = false;
      }

      //balancing logic (with hysteresis)
      if(g_charging_flag){
          //balance only when charger connected
          if(!g_balance_active_flag && abs(hw_getCell1Voltage()-hw_getCell2Voltage()) > BAT_BAL_START_THR){
              //if imbalance is larger than thrashold, set balancing flag
              g_balance_active_flag = true;
          }
          if(g_balance_active_flag && abs(hw_getCell1Voltage()-hw_getCell2Voltage()) < BAT_BAL_STOP_THR){
              //if balancing is active and cells reached close enough point, stop balancing
              g_balance_active_flag = false;
          }
      }
      else{
          g_balance_active_flag = false;
      }

      //balancing on off logic
      if(g_balance_active_flag){
          if(hw_getCell1Voltage() > hw_getCell2Voltage()){
              //cell 1 is higher, turn balancing on on cell 1
              hw_bal1(true);
          }
          else{
              //cell 2 is higher
              hw_bal2(true);
          }
      }
      else{
          hw_bal1(false);
          hw_bal2(false);
      }



  }
  /* USER CODE END analog_task_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
