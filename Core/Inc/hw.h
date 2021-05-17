//
// Created by Matej Planinšek on 10/05/2021.
//

//hardware specific functions for interfacing with on-board peripherals

// "tmc" is short for TMC2208 stepper motor dirver
// "esp" is short for ESP8266 wifi modem

#ifndef ROLLERBLINDS_FW_HW_H
#define ROLLERBLINDS_FW_HW_H

#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include "gpio.h"

//############ LOW LEVEL GPIO MANIPULATION FUNCTIONS ##########
//output functions
void hw_redLed(bool state);
void hw_blueLed(bool state);
void hw_espPower(bool state);
void hw_tmcPower(bool state);
void hw_auxGpio(bool state);
void hw_tmcDir(bool state);
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
//###########################################################



#endif //ROLLERBLINDS_FW_HW_H
