/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "App/turretStates.h"
#include "Sensors/encoders.h"

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

/* USER CODE END Variables */
/* Definitions for speedUpdateTask */
osThreadId_t speedUpdateTaskHandle;
const osThreadAttr_t speedUpdateTask_attributes = {
  .name = "speedUpdateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for toggleStateLEDTask */
osThreadId_t toggleStateLEDTaskHandle;
const osThreadAttr_t toggleStateLEDTask_attributes = {
  .name = "toggleStateLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pollModbusTask */
osThreadId_t pollModbusTaskHandle;
const osThreadAttr_t pollModbusTask_attributes = {
  .name = "pollModbusTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for checkIfTimeoutTask */
osThreadId_t checkIfTimeoutTaskHandle;
const osThreadAttr_t checkIfTimeoutTask_attributes = {
  .name = "checkIfTimeoutTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSpeedUpdateTask(void *argument);
void StartToggleStateLEDTask(void *argument);
void StartPollModbusTask(void *argument);
void StartCheckIfTimeoutTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of speedUpdateTask */
  speedUpdateTaskHandle = osThreadNew(StartSpeedUpdateTask, NULL, &speedUpdateTask_attributes);

  /* creation of toggleStateLEDTask */
  toggleStateLEDTaskHandle = osThreadNew(StartToggleStateLEDTask, NULL, &toggleStateLEDTask_attributes);

  /* creation of pollModbusTask */
  pollModbusTaskHandle = osThreadNew(StartPollModbusTask, NULL, &pollModbusTask_attributes);

  /* creation of checkIfTimeoutTask */
  checkIfTimeoutTaskHandle = osThreadNew(StartCheckIfTimeoutTask, NULL, &checkIfTimeoutTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSpeedUpdateTask */
/**
 * @brief  Function implementing the speedUpdateTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSpeedUpdateTask */
void StartSpeedUpdateTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartSpeedUpdateTask */
  /* Infinite loop */
  for (;;)
  {
    encodersUpdate();
    osDelay(10);
  }
  /* USER CODE END StartSpeedUpdateTask */
}

/* USER CODE BEGIN Header_StartToggleStateLEDTask */
/**
 * @brief Function implementing the toggleStateLEDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartToggleStateLEDTask */
void StartToggleStateLEDTask(void *argument)
{
  /* USER CODE BEGIN StartToggleStateLEDTask */
  /* Infinite loop */
  for (;;)
  {
    updateTurretStateLamp();
    osDelay(LAMP_SWITCHING_WHEN_FAULT_MS);
  }
  /* USER CODE END StartToggleStateLEDTask */
}

/* USER CODE BEGIN Header_StartPollModbusTask */
/**
 * @brief Function implementing the pollModbusTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPollModbusTask */
void StartPollModbusTask(void *argument)
{
  /* USER CODE BEGIN StartPollModbusTask */
  /* Infinite loop */
  for (;;)
  {
#if !(COMM_MODE & COMM_MODE_BIT_FRWD_USB_BT)
    eMBPoll();
#endif
    osDelay(1);
  }
  /* USER CODE END StartPollModbusTask */
}

/* USER CODE BEGIN Header_StartCheckIfTimeoutTask */
/**
 * @brief Function implementing the checkIfTimeoutTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCheckIfTimeoutTask */
void StartCheckIfTimeoutTask(void *argument)
{
  /* USER CODE BEGIN StartCheckIfTimeoutTask */
  /* Infinite loop */
  for (;;)
  {
#if !(COMM_MODE & COMM_MODE_BIT_FRWD_USB_BT)
    checkIfCommTimeout();
#endif
    osDelay(500);
  }
  /* USER CODE END StartCheckIfTimeoutTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

