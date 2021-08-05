//
// Created by Matej Planinšek on 10/05/2021.
//

//hardware specific functions for interfacing with on-board peripherals

// "tmc" is short for TMC2208 stepper motor dirver
// "esp" is short for ESP8266 wifi modem
// "stm" is short for STM32 MACU this code is running on
// "SpS" is short for stepper steps per second speed

#ifndef ROLLERBLINDS_FW_HW_H
#define ROLLERBLINDS_FW_HW_H

#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "rtc.h"
#include "usart.h"
#include <stdlib.h>
#include "main.h"






//############ LOW LEVEL GPIO MANIPULATION FUNCTIONS ##########
//output functions
void hw_redLed(bool state);
void hw_blueLed(bool state);
void hw_espPower(bool state);
void hw_tmcPower(bool state);
void hw_auxGpio(bool state);
void hw_tmcDir(bool state); //dont use this to change direction. use tmc_dir()
void hw_tmcIoSply(bool state);
void hw_tmcEnable(bool state);
void hw_bal1(bool state);
void hw_bal2(bool state);
void hw_espWake(bool state);

//input functions
bool hw_sw1();
bool hw_sw2();
bool hw_sw3();
bool hw_tmcGetDiag();
bool hw_tmcGetIndex();
bool hw_vbusPresent();
//###########################################################

//############ ADC FUNCTIONS ##########
#define ADC_NUM_CH 3
#define ADC_IDX_CELL1 1
#define ADC_IDX_CELL2 0
#define ADC_IDX_VBUS 2
#define ADC_REF 3300
#define ADC_MAX_VAL 4095
#define ADC_CELL1_COEF 1.268046 //raw reading to mV: 1/0.635514 divider coef * ADC_REF * 1/4095
#define ADC_CELL2_COEF 2.210951 //raw reading to mV: 1/0.364486 divider coef * ADC_REF * 1/4095
#define ADC_VBUS_COEF 1.268046
#define LOW_CELL_VOLT_THR 3000
#define LOW_CELL_VOLT_REC 3200
#define BAT_BAL_START_THR 40
#define BAT_BAL_STOP_THR 10
#define BAT_FULL_CHRG_THR 4150
#define VBUS_PRESENT_THR 4000

#define STATUS_BAT_DEAD 33



void hw_adcStart();
void hw_adcStop();
uint32_t hw_getCell1Voltage();
uint32_t hw_getCell2Voltage();
uint32_t hw_getPackVoltage();
uint32_t hw_getVbusVoltage();
uint8_t hw_getSoc();
//#####################################

//############ STEPPER POSITIONING FUNCTIONS ##########
//reload is timer value at which timer resets to 0
#define STP_TIM_MIN_RELOAD 40 //minimal timer reload value (max speed / pulse per second)
#define STP_TIM_MAX_RELOAD 65000 //max timer reload value (min speed / pps)
#define STP_TIM_PULSE 20
#define STP_TIMER_CLK 2000000L
#define TMC_MAX_VEL 5000
#define TMC_MIN_VEL 100
#define TMC_CRUISE_VEL 3500
#define TMC_VEL_CHNG_PER_MS 10 //velocity can change 100 stpPerSec every millisecond
#define TMC_POS_CTRL_RAMP_DOWN_LEN 1000 //speed ramping region in steps for positional controll
void tmc_direction(bool dir);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void tmc_startStepGen();
void tmc_stopStepGen();
bool tmc_setSpS(uint32_t stpPerSec);
bool tmc_commandVelocity(int32_t stpPerSec);
bool tmc_commandPosition(int32_t position);
bool tmc_posCtrlActive();

void tmc_commandPositionPercent(uint8_t posPercent);
uint8_t tmc_getPositionPercent();

#define POS_CLOSE_ENOUGH 20 //threshold for detecting closed/open state, compared to fixed up/down position
#define G_POS_DOWN 2
#define G_POS_UP 1
#define G_POS_UNKNOWN 0
//#########################################################


// ########## SLEEP AND RTC FUNCTIONS ###################
#define RTC_WAKE_TIME 5000
#define RTC_TICKS_PER_S 2048
#define TIMETABLE_REFRESH_PERIOD_SEC 900
void hw_sleep();
void hw_configClockAfterSleep();
void hw_setRtcTime(uint8_t h, uint8_t m, uint8_t s);
void hw_setRtcDate(uint8_t date, uint8_t month, uint8_t year);
uint8_t hw_getHour();
uint8_t hw_getMinute();
uint8_t hw_getSecond();
uint8_t hw_getDay();
uint8_t hw_getMonth();
uint8_t hw_getYear();
bool hw_gpioConfigForSleep();
bool hw_gpioConfigForAwake();
void hw_setRtcFromCompileTime();
void hw_inhibitSleep(bool state);
void hw_inhibitSleepReset();
uint32_t hw_getTimecode(uint32_t yr, uint8_t mnt, uint8_t day, uint8_t hr, uint8_t min, uint8_t sec);


//#######################################################

//######################### ESP COMMS ###################
#define COMM1_LEN 10 //see esp comms description in esp task
#define COMM2_LEN 13 //see esp comms description in esp task

#define ESP_DATARECV_TIMEOUT 10000

bool comm2_valid(uint8_t *comm2);
bool comm2_RtcRefreshIncluded(uint8_t *comm2);

#define COMM2_OPEN_HR 0
#define COMM2_OPEN_MIN 1
#define COMM2_CLOSE_HR 2
#define COMM2_CLOSE_MIN 3
#define COMM2_RTC_REFRESH_HR 4
#define COMM2_RTC_REFRESH_MIN 5
#define COMM2_RTC_REFRESH_SEC 6
#define COMM2_RTC_REFRESH_DATE 7
#define COMM2_RTC_REFRESH_MONTH 8
#define COMM2_RTC_REFRESH_YEAR 9

uint8_t comm2_getData(uint8_t *comm2, uint8_t dataPos);

void hw_setRtcFromComm2(uint8_t *comm2);




//#######################################################

//################## DEBUG ##############################
void dbg_debugPrint(uint8_t print [32]);
//#######################################################



uint32_t dump(uint32_t val);
#endif //ROLLERBLINDS_FW_HW_H
