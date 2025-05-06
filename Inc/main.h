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
#define COMM_BUFFER_SIZE 128
#define ELEV_MOTOR_ENCODER_RESOLUTION 20
#define ROT_MOTOR_ENCODER_RESOLUTION 20
#define MODBUS_TIM_PRESCALER 95
#define BLUETOOTH_DATA_BPS 9600
#define MODBUS_TIM_PERIOD 49
#define IWDG_RELOAD 999
#define BLACKPILL_LED_Pin GPIO_PIN_13
#define BLACKPILL_LED_GPIO_Port GPIOC
#define BLACKPILL_KEY_Pin GPIO_PIN_0
#define BLACKPILL_KEY_GPIO_Port GPIOA
#define INPUT_VOLTAGE_Pin GPIO_PIN_1
#define INPUT_VOLTAGE_GPIO_Port GPIOA
#define TURRET_DISABLE_SIGNAL_Pin GPIO_PIN_2
#define TURRET_DISABLE_SIGNAL_GPIO_Port GPIOA
#define TURRET_DISABLE_SIGNAL_EXTI_IRQn EXTI2_IRQn
#define INPUT_CURRENT_Pin GPIO_PIN_3
#define INPUT_CURRENT_GPIO_Port GPIOA
#define MOTOR1_CURRENT_Pin GPIO_PIN_0
#define MOTOR1_CURRENT_GPIO_Port GPIOB
#define MOTOR2_CURRENT_Pin GPIO_PIN_1
#define MOTOR2_CURRENT_GPIO_Port GPIOB
#define TURRET_STATE_LAMP_Pin GPIO_PIN_10
#define TURRET_STATE_LAMP_GPIO_Port GPIOB
#define TURRET_RELOAD_Pin GPIO_PIN_14
#define TURRET_RELOAD_GPIO_Port GPIOB
#define TURRET_TRIGGER_Pin GPIO_PIN_15
#define TURRET_TRIGGER_GPIO_Port GPIOB
#define BLUETOOTH_STATE_Pin GPIO_PIN_8
#define BLUETOOTH_STATE_GPIO_Port GPIOA
#define BLUETOOTH_TX_Pin GPIO_PIN_9
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_10
#define BLUETOOTH_RX_GPIO_Port GPIOA
#define ROT_ENCODER_B_Pin GPIO_PIN_15
#define ROT_ENCODER_B_GPIO_Port GPIOA
#define ROT_ENCODER_A_Pin GPIO_PIN_3
#define ROT_ENCODER_A_GPIO_Port GPIOB
#define ELEV_ENCODER_B_Pin GPIO_PIN_6
#define ELEV_ENCODER_B_GPIO_Port GPIOB
#define ELEV_ENCODER_A_Pin GPIO_PIN_7
#define ELEV_ENCODER_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

    ////////////////////////////////////////////////////////////////////////////////
    //! Comm mode related defines

#define COMM_MODE 0x06

#define COMM_MODE_BIT_ECHO_USB_APP 0x01 // Echo every data USB->Application to USB
#define COMM_MODE_BIT_ECHO_BT_APP 0x02  // Echo every data BT->Application to USB
#define COMM_MODE_BIT_ECHO_APP_BT 0x04  // Echo every data Application->BT to USB
#define COMM_MODE_BIT_FRWD_USB_BT 0x08  // Forward every data USB->BT

    ////////////////////////////////////////////////////////////////////////////////
    //! Modbus related defines

#define MODBUS_SLAVE_ID 0x01
#define MODBUS_COMM_TIMEOUT_ERROR_MS 0xFFFFFFFF //!< @note set timeout if no modbus frame received
#define LAMP_SWITCHING_WHEN_FAULT_MS 200        //!< TURRET_STATE_LAMP toggling when fault detected
#define DEBUG_USB_PRINT_MS 250                  //!< USB print period in ms

    ////////////////////////////////////////////////////////////////////////////////

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
