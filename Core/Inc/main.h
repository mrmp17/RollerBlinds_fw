/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CELL2_ADC_Pin GPIO_PIN_0
#define CELL2_ADC_GPIO_Port GPIOA
#define CELL1_ADC_Pin GPIO_PIN_1
#define CELL1_ADC_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define VBUS_PRESENT_Pin GPIO_PIN_4
#define VBUS_PRESENT_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_6
#define LED_BLUE_GPIO_Port GPIOA
#define AUX_GPIO_Pin GPIO_PIN_7
#define AUX_GPIO_GPIO_Port GPIOA
#define ESP_WAKE_Pin GPIO_PIN_12
#define ESP_WAKE_GPIO_Port GPIOB
#define ESP_PWR_CTRL_Pin GPIO_PIN_13
#define ESP_PWR_CTRL_GPIO_Port GPIOB
#define SW_3_Pin GPIO_PIN_14
#define SW_3_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_15
#define SW_2_GPIO_Port GPIOB
#define SW_1_Pin GPIO_PIN_8
#define SW_1_GPIO_Port GPIOA
#define BAL2_Pin GPIO_PIN_11
#define BAL2_GPIO_Port GPIOA
#define BAL1_Pin GPIO_PIN_12
#define BAL1_GPIO_Port GPIOA
#define TMC_STEP_Pin GPIO_PIN_15
#define TMC_STEP_GPIO_Port GPIOA
#define TMC_DIR_Pin GPIO_PIN_3
#define TMC_DIR_GPIO_Port GPIOB
#define TMC_DIAG_Pin GPIO_PIN_4
#define TMC_DIAG_GPIO_Port GPIOB
#define TMC_SPLYIO_Pin GPIO_PIN_5
#define TMC_SPLYIO_GPIO_Port GPIOB
#define TMC_UART_Pin GPIO_PIN_6
#define TMC_UART_GPIO_Port GPIOB
#define TMC_INDEX_Pin GPIO_PIN_7
#define TMC_INDEX_GPIO_Port GPIOB
#define TMC_EN_Pin GPIO_PIN_8
#define TMC_EN_GPIO_Port GPIOB
#define TMC_SPLY_SW_Pin GPIO_PIN_9
#define TMC_SPLY_SW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
