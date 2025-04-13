/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define USB_BUFFER_RX 64
#define USB_BUFFER_TX 64
#define IWDG_RELOAD 999
#define TIM11_PRESCALER 95
#define TIM11_PERIOD 49
#define BLUETOOTH_DATA_BPS 9600
#define USART1_BUFFER_RX 64
#define USART1_BUFFER_TX 64
#define BLACKPILL_LED_Pin GPIO_PIN_13
#define BLACKPILL_LED_GPIO_Port GPIOC
#define BLACKPILL_KEY_Pin GPIO_PIN_0
#define BLACKPILL_KEY_GPIO_Port GPIOA
#define BLUETOOTH_STATE_Pin GPIO_PIN_8
#define BLUETOOTH_STATE_GPIO_Port GPIOA
#define BLUETOOTH_TX_Pin GPIO_PIN_9
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_10
#define BLUETOOTH_RX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define USB_BLUETOOTH_DIRECT 0 // USB to Bluetooth direct communication

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
