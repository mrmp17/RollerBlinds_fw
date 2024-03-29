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
extern volatile int32_t g_vel_act; //actual motor velocity global variable from hw.c file
extern volatile int32_t g_vel_cmd; //global velocity command variable from hw.c file

extern volatile int32_t g_steps_abs; //global variables for absolute step counter and stepper direction from hw.c file

extern volatile int32_t g_pos_cmd; //global position command variable from hw.c file
extern volatile bool g_pos_ctrl_active; //global positional controll active flag from hw.c file

extern volatile bool g_charging_flag; //global charging flag from hw.c file
extern volatile bool g_low_battery_flag; //global low batterz flag from hw.c
extern volatile bool g_balance_active_flag; //global balance active flag from hw.c
extern volatile bool g_fully_charged_flag; //global fullu charged flag from hw.c

extern volatile int32_t g_up_pos; //global up position from hw.c
extern volatile int32_t g_down_pos; //global down position from hw.c

extern volatile bool g_esp_comms_active; //global esp comms active flag from hw.c

//extern volatile uint8_t g_SoC; //global battery state of charge variable from hw.c
extern volatile uint8_t g_status; //global status code (indicates open/close status and errors) from hw.c
extern volatile bool g_request_rtc_refresh; //global rtc time refresh request flag from hw.c. if rtc refresh ok, esp_task resets this to 0

//set position from slider on home assistant in PERCENT  value 255 indicates no movement needed
volatile uint8_t force_position_from_wifi = 255;

//todo: set status code throughout operation

// 0: unknown
// 1: up
// 2: down
uint8_t g_blinds_position = 0;

extern uint8_t g_inhibit_sleep; //global inhibit sleep flag. Keeps looping, doesnt sleep
bool g_esp_data_ok = false; //indicates esp got data (and updated timetable if new times arrived)

//timetable structure for opening/closing times When times are received from esp, we write them in timetable
//also for rtc refresh scheduled time
//set done flag when action finished, reset one minute later (prevents repeated triggering)
struct Timetable{
    uint8_t open_hr;
    uint8_t open_min;
    uint8_t close_hr;
    uint8_t close_min;
    bool open_done;
    bool close_done;
    uint8_t timetable_refresh_minute;
};
struct Timetable timetable;





/* USER CODE END Variables */
osThreadId misc_taskHandle;
osThreadId tmc_taskHandle;
osThreadId main_logic_taskHandle;
osThreadId esp_taskHandle;
osThreadId analog_taskHandle;

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

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
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
  osThreadDef(misc_task, misc_task_entry, osPriorityNormal, 0, 64);
  misc_taskHandle = osThreadCreate(osThread(misc_task), NULL);

  /* definition and creation of tmc_task */
  osThreadDef(tmc_task, tmc_task_entry, osPriorityNormal, 0, 64);
  tmc_taskHandle = osThreadCreate(osThread(tmc_task), NULL);

  /* definition and creation of main_logic_task */
  osThreadDef(main_logic_task, main_logic_task_entry, osPriorityNormal, 0, 128);
  main_logic_taskHandle = osThreadCreate(osThread(main_logic_task), NULL);

  /* definition and creation of esp_task */
  osThreadDef(esp_task, esp_task_entry, osPriorityNormal, 0, 128);
  esp_taskHandle = osThreadCreate(osThread(esp_task), NULL);

  /* definition and creation of analog_task */
  osThreadDef(analog_task, analog_task_entry, osPriorityNormal, 0, 64);
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
    while(1){
        osDelay(100);
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
    for (;;) {
        //positional controll. sets desired velocity
        //no need to ramp speed up, is handeled by velocity ctrl. only need to slow down to reach the exact destination

        if (g_pos_ctrl_active) { //positional controll active / position not yet reached
            int32_t diff = g_pos_cmd - g_steps_abs;
//          else if((g_pos_cmd >= 0 && g_steps_abs > g_pos_cmd) || (g_pos_cmd < 0 && g_steps_abs < g_pos_cmd)){
//              //we have overshot destination in this case. whoops.
//              g_pos_ctrl_active = false;
//          }
            if (abs(diff) < TMC_POS_CTRL_RAMP_DOWN_LEN) { //near destinatio
                int32_t reqVel = (TMC_CRUISE_VEL * diff) /
                                 TMC_POS_CTRL_RAMP_DOWN_LEN; //required velocity. this should be capped at TMC_MIN_VEL (min velocity at which destination is reached)
                if (reqVel >= 0 && reqVel <= TMC_MIN_VEL) reqVel = TMC_MIN_VEL;
                else if (reqVel < 0 && reqVel >= -TMC_MIN_VEL) reqVel = -TMC_MIN_VEL;
                tmc_commandVelocity(reqVel);
            } else if (diff > 0) { //cruising speed, positive direction
                tmc_commandVelocity(TMC_CRUISE_VEL);
            } else { //cruising speed, negative direction
                tmc_commandVelocity(-TMC_CRUISE_VEL);
            }
            if (abs(diff) <= 5) { //5 steps to destination is close enough
                g_pos_ctrl_active = false;
                tmc_commandVelocity(0);
            }
        }

        //velocity controll
        int32_t sps_rel;
        if (g_vel_cmd != 0 || g_vel_act != 0) { //commanded and actual velocity not zero, tmc should be ON
            tmc_startStepGen();
            hw_tmcEnable(true);
            if (g_vel_cmd < g_vel_act) {
                if (g_vel_act - g_vel_cmd <= TMC_VEL_CHNG_PER_MS * TMC_TASK_DELAY) { //we can directly jump to commanded speed
                    sps_rel = g_vel_cmd;
                    g_vel_act = sps_rel;
                    if (sps_rel >= 0) {
                        tmc_setSpS(sps_rel);
                        tmc_direction(true);
                    } else {
                        tmc_setSpS(-sps_rel);
                        tmc_direction(false);
                    }

                } else { //decrease speed max allowable amount
                    sps_rel = g_vel_act - TMC_VEL_CHNG_PER_MS * TMC_TASK_DELAY;
                    g_vel_act = sps_rel;
                    if (sps_rel >= 0) {
                        tmc_setSpS(sps_rel);
                        tmc_direction(true);
                    } else {
                        tmc_setSpS(-sps_rel);
                        tmc_direction(false);
                    }
                }
            } else if (g_vel_cmd > g_vel_act) {
                if (g_vel_cmd - g_vel_act <= TMC_VEL_CHNG_PER_MS * TMC_TASK_DELAY) { //we can directly jump to commanded speed
                    sps_rel = g_vel_cmd;
                    g_vel_act = sps_rel;
                    if (sps_rel >= 0) {
                        tmc_setSpS(sps_rel);
                        tmc_direction(true);
                    } else {
                        tmc_setSpS(-sps_rel);
                        tmc_direction(false);
                    }
                } else { //increase speed max allowable amount
                    sps_rel = g_vel_act + TMC_VEL_CHNG_PER_MS * TMC_TASK_DELAY;
                    g_vel_act = sps_rel;
                    if (sps_rel >= 0) {
                        tmc_setSpS(sps_rel);
                        tmc_direction(true);
                    } else {
                        tmc_setSpS(-sps_rel);
                        tmc_direction(false);
                    }
                }
            }
        } else { //commanded and actual velocity zero, tmc should be OFF
            tmc_stopStepGen();
            hw_tmcEnable(false);
        }

        //else: nothing to do, speed is already mached
        osDelay(TMC_TASK_DELAY);
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
    for (;;) {

        //first run code

        //set timetable members to 255 (no valid open/close times)
        timetable.close_hr = 255;
        timetable.close_min = 255;
        timetable.open_hr = 255;
        timetable.open_min = 255;
        timetable.open_done = false;
        timetable.timetable_refresh_minute = 0;

//        //set timetable members to 255 (no valid open/close times)
//        timetable.close_hr = 12;
//        timetable.close_min = 04;
//        timetable.open_hr = 12;
//        timetable.open_min = 02;
//        timetable.open_done = false;
//        timetable.rtc_refresh_day = 15;
//        timetable.rtc_done = false;
//        timetable.timetable_refresh_minute = 30;
//        timetable.timetable_refresh_done = false;



        //first thing to do: set rtc from wifi
        hw_blueLed(true);
        osDelay(1000);
        //set rtc time from wifi. tries again if failed. sw cant start if this is not ok.
        //bool rtcok = false;
        //vTaskSuspend(tmc_taskHandle);
        //vTaskSuspend(misc_taskHandle);
        //vTaskSuspend(analog_taskHandle);
        bool rtcok = false; //todo: this is set to true to disable rtc time refresh on startup
        while(!rtcok){
            osDelay(100);
            g_request_rtc_refresh = true; //request rtc refresh
            g_esp_comms_active = true; //start comms with esp
            while(g_esp_comms_active){
                //osDelay(10);
            }
            if(g_esp_data_ok && !g_request_rtc_refresh){
                rtcok = true;
            }
            else{
                rtcok = false;
            }

        }
        hw_blueLed(false);
        hw_redLed(true);
        static uint8_t loopCtrl = 1;
        while(loopCtrl){

            hw_tmcPower(true);
            hw_tmcIoSply(true);
            switch(loopCtrl){
                case 0:
                    break;
                case 1:
                    //setting upper position. red led illuminated
                    hw_redLed(true);
                    hw_blueLed(false);
                    if(hw_sw1()){
                        tmc_commandVelocity(TMC_MAX_VEL); //jogging up
                    }
                    else if(hw_sw3()){
                        tmc_commandVelocity(-TMC_MAX_VEL); //jogging down
                    }
                    else{
                        tmc_commandVelocity(0);
                    }
                    if(hw_sw2()){
                        //set position button.
                        tmc_commandVelocity(0);
                        loopCtrl = 2;
                        g_up_pos = g_steps_abs;
                        osDelay(500);
                    }
                    break;
                case 2:
                    //setting lower position. blue led illuminated
                    hw_redLed(false);
                    hw_blueLed(true);
                    //reset MCU if all three buttons pressed
                    if(hw_sw1() && hw_sw2() && hw_sw3()){
                        HAL_NVIC_SystemReset();
                    }
                    if(hw_sw1()){
                        tmc_commandVelocity(TMC_MAX_VEL); //jogging up
                    }
                    else if(hw_sw3()){
                        tmc_commandVelocity(-TMC_MAX_VEL); //jogging down
                    }
                    else{
                        tmc_commandVelocity(0);
                    }

                    hw_redLed(false);

                    if(hw_sw2()){
                        //set position button.
                        tmc_commandVelocity(0);
                        g_down_pos = g_steps_abs;
                        //calibration finished, blink a few times
                        hw_redLed(false);
                        hw_blueLed(false);
                        osDelay(100);
                        hw_redLed(true);
                        hw_blueLed(true);
                        osDelay(100);
                        hw_redLed(false);
                        hw_blueLed(false);
                        osDelay(100);
                        hw_redLed(true);
                        hw_blueLed(true);
                        osDelay(100);
                        hw_redLed(false);
                        hw_blueLed(false);
                        //calibration now finished, we can exit while loop by setting loopctrl to 0
                        loopCtrl = 0;
                    }
            }

        }
        hw_tmcIoSply(false);
        hw_tmcPower(false);
        //set next timetable refresh time
        timetable.timetable_refresh_minute = hw_scheduleMinuteCompare(TIMETABLE_REFRESH_PERIOD_MIN);
        dbg_debugPrint("calib ok\n");
        // ########## END OF STARTUP MANUAL POSITION CALIBRATION

        //plan for main logic:
        //do stuff and sleep at the end of every while cycle
        //for low battery, dont cycle the loop unless charged enough (sleep in a sepparate loop)
        //on wake up, check:
        //low battery?
        //charging?
        //button presses - manual open/close?
        //time to auto open/close?
        //time to refresh rtc?
        // nope? go back to sleep

        //if low battery, go to a nested loop, stay there until conditions no longer true



        //main logic loop

        while(1){
            //just woken up:
            hw_inhibitSleepReset();

            //check if battery is dead
            uint32_t batDeadSlpCounter = 0;
            while(hw_getSoc() == 0){
                //g_status = STATUS_BAT_DEAD;
                if(batDeadSlpCounter == 0){ //send status to esp once when enetering low bat
                    g_request_rtc_refresh = false;
                    g_esp_comms_active = true;
                    while(g_esp_comms_active);
                }
                if(batDeadSlpCounter%20 == 0){
                    osDelay(100); //allow sime time for other stuff to happen once in a while
                }
                batDeadSlpCounter++;
                hw_sleep();


                //reset MCU if all three buttons pressed
                if(hw_sw1() && hw_sw2() && hw_sw3()){
                    HAL_NVIC_SystemReset();
                }
            }

            //check if charging. inhibit sleep and enable leds if it is
            if(g_charging_flag){
                hw_inhibitSleep(true);
                if(g_fully_charged_flag){
                    hw_blueLed(true);
                    hw_redLed(false);
                }
                else{
                    hw_blueLed(false);
                    hw_redLed(true);
                }
            }
            else{
                hw_inhibitSleep(false);
                hw_blueLed(false);
                hw_redLed(false);
            }


            //buttons pressed?
            if(hw_sw1() && g_blinds_position != G_POS_UP){
                hw_tmcPower(true);
                hw_tmcIoSply(true);
                osDelay(100);
                tmc_commandPosition(g_up_pos);
                while (tmc_posCtrlActive()){
                    if(hw_sw3()){
                        tmc_commandPosition(g_down_pos);
                    }
                    else if(hw_sw1()){
                        tmc_commandPosition(g_up_pos);
                    }
                }
                hw_tmcPower(false);
                hw_tmcIoSply(false);
            }
            else if(hw_sw3() && g_blinds_position != G_POS_DOWN){
                hw_tmcPower(true);
                hw_tmcIoSply(true);
                osDelay(100);
                tmc_commandPosition(g_down_pos);
                while (tmc_posCtrlActive()){
                    if(hw_sw3()){
                        tmc_commandPosition(g_down_pos);
                    }
                    else if(hw_sw1()){
                        tmc_commandPosition(g_up_pos);
                    }
                }
                hw_tmcPower(false);
                hw_tmcIoSply(false);
            }

            //refresh timetable if middle button pressed
            else if(hw_sw2()){
                g_esp_comms_active = true; //trigger comms
                timetable.timetable_refresh_minute = hw_scheduleMinuteCompare(TIMETABLE_REFRESH_PERIOD_MIN);
            }

            //reset MCU if all three buttons pressed
            if(hw_sw1() && hw_sw2() && hw_sw3()){
                HAL_NVIC_SystemReset();
            }

            //automatic RTC opening, closing

            //open
            if(hw_getHour() == timetable.open_hr && hw_getMinute() == timetable.open_min && !timetable.open_done && g_blinds_position != G_POS_UP){
                dbg_debugPrint("auto up\n");
                hw_tmcPower(true);
                hw_tmcIoSply(true);
                osDelay(100);
                tmc_commandPosition(g_up_pos);
                while (tmc_posCtrlActive()){
                    if(hw_sw3()){
                        tmc_commandPosition(g_down_pos);
                    }
                    else if(hw_sw1()){
                        tmc_commandPosition(g_up_pos);
                    }
                }
                hw_tmcPower(false);
                hw_tmcIoSply(false);
                timetable.open_done = true;
            }

            //close
            if(hw_getHour() == timetable.close_hr && hw_getMinute() == timetable.close_min && !timetable.close_done && g_blinds_position != G_POS_DOWN){
                dbg_debugPrint("auto dn\n");
                hw_tmcPower(true);
                hw_tmcIoSply(true);
                osDelay(100);
                tmc_commandPosition(g_down_pos);
                while (tmc_posCtrlActive()){
                    if(hw_sw3()){
                        tmc_commandPosition(g_down_pos);
                    }
                    else if(hw_sw1()){
                        tmc_commandPosition(g_up_pos);
                    }
                }
                hw_tmcPower(false);
                hw_tmcIoSply(false);
                timetable.close_done = true;
            }

            //manual position from home assistant
            if(force_position_from_wifi <= 100){
                dbg_debugPrint("auto prcnt\n");
                hw_tmcPower(true);
                hw_tmcIoSply(true);
                osDelay(100);
                tmc_commandPositionPercent(force_position_from_wifi);
                while (tmc_posCtrlActive()){
                    if(hw_sw3()){
                        tmc_commandPosition(g_down_pos);
                    }
                    else if(hw_sw1()){
                        tmc_commandPosition(g_up_pos);
                    }
                }
                force_position_from_wifi = 255;
                hw_tmcPower(false);
                hw_tmcIoSply(false);
            }

            //reset close_done flag
            if(timetable.close_done){
                if(hw_getMinute() != timetable.close_min){
                    timetable.close_done = false;
                }
            }
            //reset open_done flag
            if(timetable.open_done){
                if(hw_getMinute() != timetable.open_min){
                    timetable.open_done = false;
                }
            }


            if(hw_getMinute() == timetable.timetable_refresh_minute){
                g_request_rtc_refresh = true; //request rtc refresh
                g_esp_comms_active = true; //trigger comms
                timetable.timetable_refresh_minute = hw_scheduleMinuteCompare(TIMETABLE_REFRESH_PERIOD_MIN);
            }

            //not needed - rtc is refreshed every timetable refresh
//            if(hw_getDay() == timetable.rtc_refresh_day && !timetable.rtc_done){
//                g_request_rtc_refresh = false; //not needed here
//                g_esp_comms_active = true; //triger comms
//                timetable.rtc_done = true;
//            }



            //request sleep inhibit if esp comms active
            if(g_esp_comms_active){
                hw_inhibitSleep(true);
            }
            else{
                hw_inhibitSleep(false);
            }

            //set g_blinds_position
            if(abs(g_steps_abs-g_up_pos)<POS_CLOSE_ENOUGH) g_blinds_position = G_POS_UP;
            else if(abs(g_steps_abs-g_down_pos)<POS_CLOSE_ENOUGH) g_blinds_position = G_POS_DOWN;
            else g_blinds_position = G_POS_UNKNOWN;


            if(!g_inhibit_sleep){
                osDelay(10); //allow some time for other tasks
                hw_sleep();
                //osDelay(5000);
            }
        }

        //testing control logic
//        while(true){
//            //automatic RTC closing
//            if (1 && hw_getHour() == 21 && hw_getMinute() == 30 && hw_getSecond() < 20) {
//                //closing time
//                hw_tmcPower(true);
//                hw_tmcIoSply(true);
//                hw_redLed(true);
//                tmc_commandPosition(g_down_pos);
//                while (tmc_posCtrlActive());
//            }
//
//                //automatic RTC opening
//            else if (1 && hw_getHour() == 9 && hw_getMinute() == 5 && hw_getSecond() < 20) {
//                //opening time
//                hw_tmcPower(true);
//                hw_tmcIoSply(true);
//                hw_redLed(true);
//                tmc_commandPosition(g_up_pos);
//                while (tmc_posCtrlActive());
//            }
//            else if (hw_sw1()) {
//                //gor
//                hw_tmcPower(true);
//                hw_tmcIoSply(true);
//                hw_redLed(true);
//                tmc_commandPosition(g_up_pos);
//                while (tmc_posCtrlActive());
//
//            }
//            else if (hw_sw2()) {
//            }
//            else if (hw_sw3()) {
//                //dol
//                hw_tmcPower(true);
//                hw_tmcIoSply(true);
//                hw_redLed(true);
//                tmc_commandPosition(g_down_pos);
//                while (tmc_posCtrlActive());
//
//            } else {
//
//            }
//
//            if (g_charging_flag) {
//                hw_blueLed(true);
//            } else {
//                hw_blueLed(false);
//                hw_redLed(false);
//                //osDelay(5000);
//                hw_sleep();
//                //hw_redLed(true);
//
//            }
//        }




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
    for (;;) {
        osDelay(1);
        static uint8_t slider_pos_prev = 255; //255 only at boot
        // communication with ESP8266 module:
        // - STM turns on ESP power
        // - STM sends status(error, battery) and clock refresh request to ESP (frame COMM1)
        // - ESP connects to wifi, gets opening/closing time and time/date if requested
        // - ESP sends data to STM (frame COMM2)
        // - STM shuts off power to ESP

        // STM implements timeouts on communication
        //all frames are arrays of bytes (fixed length)

        // COMM1 frame: [bStatus, bPosition, bBatteryPercent, bBatteryDelta, bOpenHr, bOpenMin, bCloseHr, bCloseMin, bRequestTimeRefresh, bSum] len=10
        // COMM2 frame: [bOpenHr, bOpenMin, bCloseHr, bCloseMin, bTimeNowHr, bTimeNowMin, bTimeNowSec, bDateNowDate, bDateNowMonth, bDateNowYear, bEnableAuto, bManualPosition, bSum] len=13

        //sum is cheksup byte. sum of all previous bytes + 1 (with normal uintt8_t overflow)
        // if time values not available or requested set bytes to 0xFF

        if(g_esp_comms_active){
            dbg_debugPrint("tmtbl rfrsh strt\n");
            g_esp_data_ok = false;
            uint8_t comm2_data[COMM2_LEN] = {0};
            //main logic comanded
            hw_espPower(true); //enable esp power //todo: commented out for testing
            osDelay(100); //wait for esp to wake up and start running

            //purge uart receiver
            uint8_t shitdata[3];
            HAL_UART_Receive(&hlpuart1, shitdata, 3, 100);

            uint8_t comm1[COMM1_LEN] = {g_status, tmc_getPositionPercent(), hw_getSoc(), abs((int32_t)hw_getCell2Voltage()-(int32_t)hw_getCell1Voltage()), timetable.open_hr, timetable.open_min, timetable.close_hr, timetable.close_min, g_request_rtc_refresh, 0};
            uint8_t checksum = 0;
            for(uint8_t n = 0 ; n<COMM1_LEN ; n++){ //calculate checksum
                checksum += comm1[n];
            }
            checksum += 1;
            comm1[COMM1_LEN-1] = checksum;
            //HAL_GPIO_TogglePin(AUX_GPIO_GPIO_Port, AUX_GPIO_Pin);
            volatile HAL_StatusTypeDef ok = HAL_UART_Transmit(&hlpuart1, comm1, COMM1_LEN, 100); //transmit first frame
            for(uint8_t i = 0 ; i<COMM2_LEN ; i++){ //clear array for new data
                comm2_data[i] = 0;
            }
            //vTaskSuspendAll();
            //volatile bool gt = __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_FE);
            //gt = __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_ORE);
            //gt = __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_PE);
            //gt = __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_ABRE);

            //HAL_UART_Receive_DMA(&hlpuart1, comm2_data, 1);
            //HAL_GPIO_TogglePin(AUX_GPIO_GPIO_Port, AUX_GPIO_Pin);
            //while(__HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx) != 0);
            //volatile bool gt = __HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE);
//            __HAL_UART_CLEAR_FEFLAG(&hlpuart1);
//            __HAL_UART_CLEAR_OREFLAG(&hlpuart1);
//            __HAL_UART_CLEAR_PEFLAG(&hlpuart1);
//            __HAL_UART_CLEAR_NEFLAG(&hlpuart1);
//            __HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_RXNE);
//            __HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_TC);
//            __HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_RXNE);
            HAL_StatusTypeDef recvStat = HAL_UART_Receive_DMA(&hlpuart1, comm2_data, COMM2_LEN);
            //HAL_StatusTypeDef recvStat = HAL_UART_Receive(&hlpuart1, comm2_data, COMM2_LEN, ESP_DATARECV_TIMEOUT);
            //xTaskResumeAll();
            uint32_t startUartRecv = HAL_GetTick();
            while(__HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx) != 0 && HAL_GetTick() - startUartRecv < ESP_DATARECV_TIMEOUT);
            if(recvStat == HAL_OK){ //wait for data received
                //received data before timeout.
                //data is now transfered to array and can pe read by main logic (which checks for validity)
                hw_espPower(false); //turn power off
                //g_esp_comms_active = false; //todo: correct placement of this??
                //transfer data to timetable structure
                if(comm2_valid(comm2_data)){
                    //transfer timetable data
                    if(comm2_getData(comm2_data, COMM2_ENABLE_AUTO)){
                        timetable.open_hr = comm2_getData(comm2_data, COMM2_OPEN_HR);
                        timetable.open_min = comm2_getData(comm2_data, COMM2_OPEN_MIN);
                        timetable.close_hr = comm2_getData(comm2_data, COMM2_CLOSE_HR);
                        timetable.close_min = comm2_getData(comm2_data, COMM2_CLOSE_MIN);
                    }
                    //auto disabled, set to 255 to never auto close/open
                    else{
                        timetable.open_hr = 255;
                        timetable.open_min = 255;
                        timetable.close_hr = 255;
                        timetable.close_min = 255;
                    }
                    dbg_debugPrint("tmtbl rfrsh ok\n");
                    g_esp_data_ok = true;

                    //manual positioning with slider
                    if(comm2_getData(comm2_data, COMM2_MANUAL_POS) != slider_pos_prev && slider_pos_prev != 255){
                        force_position_from_wifi = comm2_getData(comm2_data, COMM2_MANUAL_POS);
                        dbg_debugPrint("man pos writen\n");
                    }
                    else{
                        //force_position_from_wifi = 255;
                    }
                    slider_pos_prev = comm2_getData(comm2_data, COMM2_MANUAL_POS);

                    if(comm2_RtcRefreshIncluded(comm2_data)){
                        hw_setRtcFromComm2(comm2_data);
                        g_request_rtc_refresh = false; //reset to let main logic know that rtc refresh successfull
                        dbg_debugPrint("rtc rfrsh ok\n");
                    }
                }
                else{
                    dbg_debugPrint("crpted\n");
                }
            }
            else{
                dbg_debugPrint("tmout\n");
                //data was not received, end communication
                hw_espPower(false); //turn power off
                //g_esp_comms_active = false;
            }
            g_esp_comms_active = false;
        }
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
    for (;;) {
        //low battery flag logic (whit hysteresis)
        if (!g_low_battery_flag &&
            (hw_getCell1Voltage() < LOW_CELL_VOLT_THR || hw_getCell2Voltage() < LOW_CELL_VOLT_THR)) {
            //if low battery flag not set and one cell dips below LOW_CELL_VOLT_THR, set low bat flag
            g_low_battery_flag = true;
        }
        if (g_low_battery_flag &&
            (hw_getCell1Voltage() > LOW_CELL_VOLT_REC || hw_getCell2Voltage() > LOW_CELL_VOLT_REC)) {
            //if low battery flag is set and both cells rise above LOW_CELL_VOLT_REC, reset low bat flag
            g_low_battery_flag = false;
        }

        //charging flag logic
        if (hw_vbusPresent()) {
            g_charging_flag = true;
            if (hw_getCell1Voltage() > BAT_FULL_CHRG_THR && hw_getCell2Voltage() > BAT_FULL_CHRG_THR) {
                //battery is fully charged
                g_fully_charged_flag = true;
            }
        } else {
            g_charging_flag = false;
            g_fully_charged_flag = false;
        }

        //balancing logic (with hysteresis)
        if (g_charging_flag) {
            //balance only when charger connected
            if (!g_balance_active_flag &&
                abs((int32_t) hw_getCell1Voltage() - (int32_t) hw_getCell2Voltage()) > BAT_BAL_START_THR) {
                //if imbalance is larger than thrashold, set balancing flag
                g_balance_active_flag = true;
            }
            if (g_balance_active_flag &&
                abs((int32_t) hw_getCell1Voltage() - (int32_t) hw_getCell2Voltage()) < BAT_BAL_STOP_THR) {
                //if balancing is active and cells reached close enough point, stop balancing
                g_balance_active_flag = false;
            }
        } else {
            g_balance_active_flag = false;
        }

        //balancing on off logic
        if (g_balance_active_flag) {
            if (hw_getCell1Voltage() > hw_getCell2Voltage()) {
                //cell 1 is higher, turn balancing on on cell 1
                hw_bal1(true);
            } else {
                //cell 2 is higher
                hw_bal2(true);
            }
        } else {
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
