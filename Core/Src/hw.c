//
// Created by Matej Planinšek on 10/05/2021.
//
#include "hw.h"


//set red led state
void hw_redLed(bool state){
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set blue led state
void hw_blueLed(bool state){
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set ESP power state
void hw_espPower(bool state){
    HAL_GPIO_WritePin(ESP_PWR_CTRL_GPIO_Port, ESP_PWR_CTRL_Pin, state?GPIO_PIN_RESET:GPIO_PIN_SET);
    //logic level on gpio is inverted
}

// set tmc power state
void hw_tmcPower(bool state){
    HAL_GPIO_WritePin(TMC_SPLY_SW_GPIO_Port, TMC_SPLY_SW_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set aux gpio state
void hw_auxGpio(bool state){
    HAL_GPIO_WritePin(AUX_GPIO_GPIO_Port, AUX_GPIO_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set tmc dir state
void hw_tmcDir(bool state){
    HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set tmc IO sply state
void hw_tmcIoSply(bool state){
    HAL_GPIO_WritePin(TMC_SPLYIO_GPIO_Port, TMC_SPLYIO_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set tmc enable  state
void hw_tmcEnable(bool state){
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, state?GPIO_PIN_RESET:GPIO_PIN_SET);
}

//set balancing state on cell 1
void hw_bal1(bool state){
    HAL_GPIO_WritePin(BAL1_GPIO_Port, BAL1_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set balancing state on cell 1
void hw_bal2(bool state){
    HAL_GPIO_WritePin(BAL2_GPIO_Port, BAL2_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}

//set esp wake pin state
void hw_espWake(bool state){
    HAL_GPIO_WritePin(ESP_WAKE_GPIO_Port, ESP_WAKE_Pin, state?GPIO_PIN_SET:GPIO_PIN_RESET);
}




//get sw1 state
bool hw_sw1(){
    if(HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin)==GPIO_PIN_RESET) return true;
    else return false;
}

//get sw2 state
bool hw_sw2(){
    if(HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin)==GPIO_PIN_RESET) return true;
    else return false;
}

//get sw3 state
bool hw_sw3(){
    if(HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin)==GPIO_PIN_RESET) return true;
    else return false;
}

//get tmc diag state
bool hw_tmcGetDiag(){
    if(HAL_GPIO_ReadPin(TMC_DIAG_GPIO_Port, TMC_DIAG_Pin)==GPIO_PIN_SET) return true;
    else return false;
}

//get tmc index state
bool hw_tmcGetIndex(){
    if(HAL_GPIO_ReadPin(TMC_INDEX_GPIO_Port, TMC_INDEX_Pin)==GPIO_PIN_SET) return true;
    else return false;
}

//get status of vbus voltage. true if usb vbus present. (this could be connected to ADC in the future)
bool hw_vbusPresent(){
    if(HAL_GPIO_ReadPin(VBUS_PRESENT_GPIO_Port, VBUS_PRESENT_Pin)==GPIO_PIN_SET) return true;
    else return false;
}




uint32_t adcBuffer[ADC_NUM_CH] = {0};
void hw_adcStart(){
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc, adcBuffer, ADC_NUM_CH);
}

void hw_adcStop(){
    HAL_ADC_Stop_DMA(&hadc);
}

//returns cell 1 voltage in mV
uint32_t hw_getCell1Voltage(){
    //return adcBuffer[ADC_IDX_CELL1];
    return adcBuffer[ADC_IDX_CELL1]*ADC_CELL1_COEF;
}

//returns cell 2 voltage in mV
uint32_t hw_getCell2Voltage(){
    hw_getPackVoltage()-hw_getCell1Voltage();

}

//returns pack (cell1 in series with cell2) voltage in mV
uint32_t hw_getPackVoltage(){
    return adcBuffer[ADC_IDX_CELL2]*ADC_CELL2_COEF;
}


int32_t g_steps_abs = 0; //global variables for absolute step counter and stepper direction
bool g_tmc_direction = false;

//set tmc direction
void tmc_direction(bool dir){
    hw_tmcDir(dir);
    g_tmc_direction = dir;
}

//timer pwm pulse finished callback. is called at counter compare event when couter=pulse value (rising edge)
//we use this to count step commands to tmc stepper driver for positional control
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim2){ //this is our pwm timer
        g_tmc_direction?g_steps_abs++:g_steps_abs--; //increment absolute step counter if direction is true, decrement if false
    }
}

//starts generating step pulses for tmc. Set correct speed & direction before calling this!
void tmc_startStepGen(){
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
}

//stops generating step pulses for tmc.
void tmc_stopStepGen(){
    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
}

int32_t g_vel_act = 0; //actual motor velocity global variable
//sets step pulse frequency for correct steps per second - stepper motor velocity
//returns true if parameter in range (set to nearest possible freq)
//this is microstepping, not full steps!
bool tmc_setSpS(uint32_t stpPerSec){
    // stpPerSec is equal to timer frequency at current reload and pulse value
    // timer freq equal to clk/(reload+1)
    // we want reload = (clk/freq) -1, freq=stpPerSec
    //integer division should be ok here
    uint32_t rld = (STP_TIMER_CLK/stpPerSec) -1;
    bool ret = true;

    //todo: input param = 0 obnašanje??
    //cap reload value to valid limits. These are hard timer limits not allowable motor speed!
    if(rld > STP_TIM_MAX_RELOAD){
        rld = STP_TIM_MAX_RELOAD;
        ret = false;
    }
    else if(rld < STP_TIM_MIN_RELOAD){
        rld = STP_TIM_MIN_RELOAD;
        ret = false;
    }
    __HAL_TIM_SET_AUTORELOAD(&htim2, rld);
    return ret;
}

int32_t g_vel_cmd = 0; //global velocity command variable
//sets desired motor velocity in steps per second. Actual motor velocity can be different due to ramping (handeled in rtos task)
//returns false if set speed too high
bool tmc_commandVelocity(int32_t stpPerSec){
    if(abs(stpPerSec) > TMC_MAX_VEL) return false;

    g_vel_cmd = stpPerSec; //write comanded velocity to global variable
    return true;
}

int32_t g_pos_cmd = 0; //global position command variable
bool g_pos_ctrl_active = false; //global positional controll active flag
//sets commanded motor position (in relation to absolute step counter). returns false if this is not possible
//also sets g_pos_ctrl_active flag. flag resets when comanded position is reached
bool tmc_commandPosition(int32_t position){
    g_pos_cmd = position;
    g_pos_ctrl_active = true;
    return true;
    //todo: validity checking
}

//returns true if positional control is active (velocity is controlled by pos ctrl)
bool tmc_posCtrlActive(){
    return g_pos_ctrl_active;
}




uint32_t dump(uint32_t val){
    return 0;
}