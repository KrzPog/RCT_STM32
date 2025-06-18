/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLACKPILL_LED_GPIO_Port, BLACKPILL_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROT_DIR_Pin|TURRET_LAMP_Pin|TURRET_RELOAD_Pin|TURRET_TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ELEV_DIR_GPIO_Port, ELEV_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLACKPILL_LED_Pin */
  GPIO_InitStruct.Pin = BLACKPILL_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLACKPILL_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLACKPILL_KEY_Pin */
  GPIO_InitStruct.Pin = BLACKPILL_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BLACKPILL_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROT_DIR_Pin TURRET_LAMP_Pin TURRET_RELOAD_Pin TURRET_TRIGGER_Pin */
  GPIO_InitStruct.Pin = ROT_DIR_Pin|TURRET_LAMP_Pin|TURRET_RELOAD_Pin|TURRET_TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ELEV_DIR_Pin */
  GPIO_InitStruct.Pin = ELEV_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ELEV_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DISABLE_SIGNAL_Pin */
  GPIO_InitStruct.Pin = DISABLE_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DISABLE_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ELEV_LIMIT_MIN_Pin ELEV_LIMIT_MAX_Pin */
  GPIO_InitStruct.Pin = ELEV_LIMIT_MIN_Pin|ELEV_LIMIT_MAX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  GPIO_PinState pinState;

  switch (GPIO_Pin)
  {
  case DISABLE_SIGNAL_Pin:
    deactivateTurret();
    break;
  case ELEV_LIMIT_MIN_Pin:
    pinState = HAL_GPIO_ReadPin(ELEV_LIMIT_MIN_GPIO_Port, ELEV_LIMIT_MIN_Pin);
    if (pinState == GPIO_PIN_RESET)
      elevLimitMinReached = true; //!< Pull down logic
    else
      elevLimitMinReached = false;
    break;
  case ELEV_LIMIT_MAX_Pin:
    pinState = HAL_GPIO_ReadPin(ELEV_LIMIT_MAX_GPIO_Port, ELEV_LIMIT_MAX_Pin);
    if (pinState == GPIO_PIN_RESET)
      elevLimitMaxReached = true; //!< Pull down logic
    else
      elevLimitMaxReached = false;
    break;
  default:
    break;
  }
}

/* USER CODE END 2 */
