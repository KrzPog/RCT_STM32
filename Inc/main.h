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
#define MODBUS_TIM_PRESCALER 95
#define BLUETOOTH_USART_BPS 9600
#define ELEV_USART_BPS 9600
#define MODBUS_TIM_PERIOD 49
#define ELEV_PWM_PERIOD 999
#define ELEV_PWM_PRESCALER 3
#define COMMS_BUFFER_SIZE 128
#define ROT_PWM_PERIOD 999
#define IWDG_RELOAD 999
#define ROT_USART_BPS 9600
#define ROT_PWM_PRESCALER 3
#define BLACKPILL_LED_Pin GPIO_PIN_13
#define BLACKPILL_LED_GPIO_Port GPIOC
#define BLACKPILL_KEY_Pin GPIO_PIN_0
#define BLACKPILL_KEY_GPIO_Port GPIOA
#define ROT_PWM_Pin GPIO_PIN_1
#define ROT_PWM_GPIO_Port GPIOA
#define ROT_USART_TX_Pin GPIO_PIN_2
#define ROT_USART_TX_GPIO_Port GPIOA
#define ROT_USART_RX_Pin GPIO_PIN_3
#define ROT_USART_RX_GPIO_Port GPIOA
#define ROT_DIR_Pin GPIO_PIN_4
#define ROT_DIR_GPIO_Port GPIOA
#define LAMP_CURRENT_Pin GPIO_PIN_5
#define LAMP_CURRENT_GPIO_Port GPIOA
#define RELOAD_CURRENT_Pin GPIO_PIN_6
#define RELOAD_CURRENT_GPIO_Port GPIOA
#define TRIGGER_CURRENT_Pin GPIO_PIN_7
#define TRIGGER_CURRENT_GPIO_Port GPIOA
#define BATTERY_VOLTAGE_Pin GPIO_PIN_0
#define BATTERY_VOLTAGE_GPIO_Port GPIOB
#define ELEV_DIR_Pin GPIO_PIN_1
#define ELEV_DIR_GPIO_Port GPIOB
#define ELEV_PWM_Pin GPIO_PIN_10
#define ELEV_PWM_GPIO_Port GPIOB
#define DISABLE_SIGNAL_Pin GPIO_PIN_12
#define DISABLE_SIGNAL_GPIO_Port GPIOB
#define DISABLE_SIGNAL_EXTI_IRQn EXTI15_10_IRQn
#define ELEV_LIMIT_MIN_Pin GPIO_PIN_13
#define ELEV_LIMIT_MIN_GPIO_Port GPIOB
#define ELEV_LIMIT_MIN_EXTI_IRQn EXTI15_10_IRQn
#define ELEV_LIMIT_MAX_Pin GPIO_PIN_14
#define ELEV_LIMIT_MAX_GPIO_Port GPIOB
#define ELEV_LIMIT_MAX_EXTI_IRQn EXTI15_10_IRQn
#define TURRET_LAMP_Pin GPIO_PIN_8
#define TURRET_LAMP_GPIO_Port GPIOA
#define TURRET_RELOAD_Pin GPIO_PIN_9
#define TURRET_RELOAD_GPIO_Port GPIOA
#define TURRET_TRIGGER_Pin GPIO_PIN_10
#define TURRET_TRIGGER_GPIO_Port GPIOA
#define BLUETOOTH_TX_Pin GPIO_PIN_15
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_3
#define BLUETOOTH_RX_GPIO_Port GPIOB
#define ROT_ENCODER_A_Pin GPIO_PIN_4
#define ROT_ENCODER_A_GPIO_Port GPIOB
#define ROT_ENCODER_B_Pin GPIO_PIN_5
#define ROT_ENCODER_B_GPIO_Port GPIOB
#define ELEV_ENCODER_A_Pin GPIO_PIN_6
#define ELEV_ENCODER_A_GPIO_Port GPIOB
#define ELEV_ENCODER_B_Pin GPIO_PIN_7
#define ELEV_ENCODER_B_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

    ////////////////////////////////////////////////////////////////////////////////
    //! Modbus related defines

#define MODBUS_SLAVE_ID 0x01
#define MODBUS_TIMEOUT_ERROR_MS 0xFFFFFFFF //!< @note set timeout if no modbus frame received
#define DEBUG_USB_PRINT_MS 250                  //!< USB print period in ms
#define ELEV_UART_ENABLED 0

    ////////////////////////////////////////////////////////////////////////////////

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
