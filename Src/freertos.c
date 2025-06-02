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

#include "usbd_cdc_if.h"
#include "freertos_tasks.h"

#include "App/turretStates.h"
#include "ModbusRegisters/reg_input.h"
#include "ModbusRegisters/reg_holding.h"
#include "Sensors/encoders.h"
#include "Sensors/analog_sensors.h"
#include "Sensors/accelerometer.h"

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
/* Definitions for task_encodersUpdate */
osThreadId_t task_encodersUpdateHandle;
const osThreadAttr_t task_encodersUpdate_attributes = {
    .name = "task_encodersUpdate",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for task_stateLEDUpdate */
osThreadId_t task_stateLEDUpdateHandle;
const osThreadAttr_t task_stateLEDUpdate_attributes = {
    .name = "task_stateLEDUpdate",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for task_pollModbus */
osThreadId_t task_pollModbusHandle;
const osThreadAttr_t task_pollModbus_attributes = {
    .name = "task_pollModbus",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for task_checkIfTimeout */
osThreadId_t task_checkIfTimeoutHandle;
const osThreadAttr_t task_checkIfTimeout_attributes = {
    .name = "task_checkIfTimeout",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for task_PID_Rot_Position */
osThreadId_t task_PID_Rot_PositionHandle;
const osThreadAttr_t task_PID_Rot_Position_attributes = {
    .name = "task_PID_Rot_Position",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for task_PID_Elev_Position */
osThreadId_t task_PID_Elev_PositionHandle;
const osThreadAttr_t task_PID_Elev_Position_attributes = {
    .name = "task_PID_Elev_Position",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for task_debugUSBPrint */
osThreadId_t task_debugUSBPrintHandle;
const osThreadAttr_t task_debugUSBPrint_attributes = {
    .name = "task_debugUSBPrint",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for task_checkAccelValues */
osThreadId_t task_checkAccelValuesHandle;
const osThreadAttr_t task_checkAccelValues_attributes = {
    .name = "task_checkAccelValues",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for task_checkAnalogSensorsData */
osThreadId_t task_checkAnalogSensorsDataHandle;
const osThreadAttr_t task_checkAnalogSensorsData_attributes = {
    .name = "task_checkAnalogSensorsData",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for task_shootingTimeLimit */
osThreadId_t task_shootingTimeLimitHandle;
const osThreadAttr_t task_shootingTimeLimit_attributes = {
    .name = "task_shootingTimeLimit",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh7,
};
/* Definitions for task_PID_Rot_Speed */
osThreadId_t task_PID_Rot_SpeedHandle;
const osThreadAttr_t task_PID_Rot_Speed_attributes = {
    .name = "task_PID_Rot_Speed",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for task_PID_Elev_Speed */
osThreadId_t task_PID_Elev_SpeedHandle;
const osThreadAttr_t task_PID_Elev_Speed_attributes = {
    .name = "task_PID_Elev_Speed",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void taskInit_encodersUpdate(void *argument);
void taskInit_stateLEDUpdate(void *argument);
void taskInit_pollModbus(void *argument);
void taskInit_checkIfTimeout(void *argument);
void taskInit_PID_Rot_Position(void *argument);
void taskInit_PID_Elev_Position(void *argument);
void taskInit_debugUSBPrint(void *argument);
void taskInit_checkAccelValues(void *argument);
void taskInit_checkAnalogSensorsData(void *argument);
void taskInit_shootingTimeLimit(void *argument);
void taskInit_PID_Rot_Speed(void *argument);
void taskInit_PID_Elev_Speed(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* creation of task_encodersUpdate */
  task_encodersUpdateHandle = osThreadNew(taskInit_encodersUpdate, NULL, &task_encodersUpdate_attributes);

  /* creation of task_stateLEDUpdate */
  task_stateLEDUpdateHandle = osThreadNew(taskInit_stateLEDUpdate, NULL, &task_stateLEDUpdate_attributes);

  /* creation of task_pollModbus */
  task_pollModbusHandle = osThreadNew(taskInit_pollModbus, NULL, &task_pollModbus_attributes);

  /* creation of task_checkIfTimeout */
  task_checkIfTimeoutHandle = osThreadNew(taskInit_checkIfTimeout, NULL, &task_checkIfTimeout_attributes);

  /* creation of task_PID_Rot_Position */
  task_PID_Rot_PositionHandle = osThreadNew(taskInit_PID_Rot_Position, NULL, &task_PID_Rot_Position_attributes);

  /* creation of task_PID_Elev_Position */
  task_PID_Elev_PositionHandle = osThreadNew(taskInit_PID_Elev_Position, NULL, &task_PID_Elev_Position_attributes);

  /* creation of task_debugUSBPrint */
  task_debugUSBPrintHandle = osThreadNew(taskInit_debugUSBPrint, NULL, &task_debugUSBPrint_attributes);

  /* creation of task_checkAccelValues */
  task_checkAccelValuesHandle = osThreadNew(taskInit_checkAccelValues, NULL, &task_checkAccelValues_attributes);

  /* creation of task_checkAnalogSensorsData */
  task_checkAnalogSensorsDataHandle = osThreadNew(taskInit_checkAnalogSensorsData, NULL, &task_checkAnalogSensorsData_attributes);

  /* creation of task_shootingTimeLimit */
  task_shootingTimeLimitHandle = osThreadNew(taskInit_shootingTimeLimit, NULL, &task_shootingTimeLimit_attributes);

  /* creation of task_PID_Rot_Speed */
  task_PID_Rot_SpeedHandle = osThreadNew(taskInit_PID_Rot_Speed, NULL, &task_PID_Rot_Speed_attributes);

  /* creation of task_PID_Elev_Speed */
  task_PID_Elev_SpeedHandle = osThreadNew(taskInit_PID_Elev_Speed, NULL, &task_PID_Elev_Speed_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_taskInit_encodersUpdate */
/**
 * @brief  Function implementing the task_encodersUpdate thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_encodersUpdate */
void taskInit_encodersUpdate(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN taskInit_encodersUpdate */

  encodersInit();

  /* Infinite loop */
  for (;;)
  {
    encoderUpdate((Encoder *)&rot_motor_encoder);
    encoderUpdate((Encoder *)&elev_motor_encoder);
    osDelay(ENCODER_SAMPLING_TIME_MS / portTICK_RATE_MS);
  }
  /* USER CODE END taskInit_encodersUpdate */
}

/* USER CODE BEGIN Header_taskInit_stateLEDUpdate */
/**
 * @brief Function implementing the task_stateLEDUpdate thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_stateLEDUpdate */
void taskInit_stateLEDUpdate(void *argument)
{
  /* USER CODE BEGIN taskInit_stateLEDUpdate */
  /* Infinite loop */
  for (;;)
  {
    stateLEDUpdate();
    osDelay(STATE_LED_SWITCHING_PERIOD_MS / portTICK_RATE_MS);
  }
  /* USER CODE END taskInit_stateLEDUpdate */
}

/* USER CODE BEGIN Header_taskInit_pollModbus */
/**
 * @brief Function implementing the task_pollModbus thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_pollModbus */
void taskInit_pollModbus(void *argument)
{
  /* USER CODE BEGIN taskInit_pollModbus */
  /* Infinite loop */
  for (;;)
  {
    eMBPoll();
    osDelay(1);
  }
  /* USER CODE END taskInit_pollModbus */
}

/* USER CODE BEGIN Header_taskInit_checkIfTimeout */
/**
 * @brief Function implementing the task_checkIfTimeout thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_checkIfTimeout */
void taskInit_checkIfTimeout(void *argument)
{
  /* USER CODE BEGIN taskInit_checkIfTimeout */
  /* Infinite loop */
  for (;;)
  {
    checkIfCommTimeout();
    osDelay(500 / portTICK_RATE_MS);
  }
  /* USER CODE END taskInit_checkIfTimeout */
}

/* USER CODE BEGIN Header_taskInit_PID_Rot_Position */
/**
 * @brief Function implementing the task_PID_Rot_Position thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_PID_Rot_Position */
void taskInit_PID_Rot_Position(void *argument)
{
  /* USER CODE BEGIN taskInit_PID_Rot_Position */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1); // delay 55 ms at the beginning of loop, read delay from macro defined in pid.h
    // here copy feedback position from REG_INPUT_ROT_POSITION to pid struct PV
    // here copy target position from REG_HOLDING_ROT_TARGET_POSITION to pid struct SP
    // here call pidUpdate() function
    // here copy from pid struct CV to speed pid struct CV
  }
  /* USER CODE END taskInit_PID_Rot_Position */
}

/* USER CODE BEGIN Header_taskInit_PID_Elev_Position */
/**
 * @brief Function implementing the task_PID_Elev_Position thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_PID_Elev_Position */
void taskInit_PID_Elev_Position(void *argument)
{
  /* USER CODE BEGIN taskInit_PID_Elev_Position */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1); // delay 55 ms at the beginning of loop, read delay from macro defined in pid.h
    // here copy feedback position from REG_INPUT_ELEV_POSITION to pid struct PV
    // here copy target position from REG_HOLDING_ELEV_TARGET_POSITION to pid struct SP
    // here call pidUpdate() function
    // here copy from pid struct CV to speed pid struct CV
  }
  /* USER CODE END taskInit_PID_Elev_Position */
}

/* USER CODE BEGIN Header_taskInit_debugUSBPrint */
/**
 * @brief Function implementing the task_debugUSBPrint thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_debugUSBPrint */
void taskInit_debugUSBPrint(void *argument)
{
  /* USER CODE BEGIN taskInit_debugUSBPrint */
  /* Infinite loop */
  for (;;)
  {
    //!< @note Purpose of this task is to print debug information to USB CDC
    // USR_Printf_USBD_CDC("\tRot [P:%5d V:%6d], Elev [P:%5d V:%6d]", (int16_t)regInput[regInpIx(REG_INPUT_ROT_POSITION)], (int16_t)regInput[regInpIx(REG_INPUT_ROT_SPEED)], (int16_t)regInput[regInpIx(REG_INPUT_ELEV_POSITION)], (int16_t)regInput[regInpIx(REG_INPUT_ELEV_SPEED)]);
    // osDelay(5 / portTICK_RATE_MS);
    // USR_Printf_USBD_CDC("\tAccel [X:%6d Y:%6d Z:%6d], Gyro [X:%6d Y:%6d Z:%6d]", (int16_t)regInput[regInpIx(REG_INPUT_ACCEL_X)], (int16_t)regInput[regInpIx(REG_INPUT_ACCEL_Y)], (int16_t)regInput[regInpIx(REG_INPUT_ACCEL_Z)], (int16_t)regInput[regInpIx(REG_INPUT_GYRO_X)], (int16_t)regInput[regInpIx(REG_INPUT_GYRO_Y)], (int16_t)regInput[regInpIx(REG_INPUT_GYRO_Z)]);
    // osDelay(5 / portTICK_RATE_MS);
    USR_Printf_USBD_CDC("\tBattery_V:%5d, Trigg_I:%5d, Reload_I:%5d, Lamp_I:%5d\r\n", (uint16_t)regInput[regInpIx(REG_INPUT_BATTERY_VOLTAGE)], (uint16_t)regInput[regInpIx(REG_INPUT_TRIGGER_CURRENT)], (uint16_t)regInput[regInpIx(REG_INPUT_RELOAD_CURRENT)], (uint16_t)regInput[regInpIx(REG_INPUT_LAMP_CURRENT)]);
    osDelay(90 / portTICK_RATE_MS);
  }
  /* USER CODE END taskInit_debugUSBPrint */
}

/* USER CODE BEGIN Header_taskInit_checkAccelValues */
/**
 * @brief Function implementing the task_checkAccelValues thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_checkAccelValues */
void taskInit_checkAccelValues(void *argument)
{
  /* USER CODE BEGIN taskInit_checkAccelValues */
  MPU6050_Init();
  /* Infinite loop */
  for (;;)
  {
    accelerometerUpdate();
    osDelay(ACCELEROMETER_SAMPLING_TIME_MS / portTICK_RATE_MS);
  }
  /* USER CODE END taskInit_checkAccelValues */
}

/* USER CODE BEGIN Header_taskInit_checkAnalogSensorsData */
/**
 * @brief Function implementing the task_checkAnalogSensorsData thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_checkAnalogSensorsData */
void taskInit_checkAnalogSensorsData(void *argument)
{
  /* USER CODE BEGIN taskInit_checkAnalogSensorsData */
  /* Infinite loop */
  for (;;)
  {
    analogSensorsUpdate();
    osDelay(ANALOG_SAMPLING_TIME_MS / portTICK_RATE_MS);
  }
  /* USER CODE END taskInit_checkAnalogSensorsData */
}

/* USER CODE BEGIN Header_taskInit_shootingTimeLimit */
/**
 * @brief Function implementing the task_shootingTimeLimit thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_shootingTimeLimit */
void taskInit_shootingTimeLimit(void *argument)
{
  /* USER CODE BEGIN taskInit_shootingTimeLimit */
  /* Infinite loop */
  for (;;)
  {
    vTaskSuspend(NULL);
    osDelay(regFlash[regFlashIx(REG_FLASH_SHOOTING_TIME_LIMIT)] / portTICK_RATE_MS);
    if (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_SHOOTING)
      shootTurretToggle();
  }
  /* USER CODE END taskInit_shootingTimeLimit */
}

/* USER CODE BEGIN Header_taskInit_PID_Rot_Speed */
/**
 * @brief Function implementing the task_PID_Rot_Speed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_PID_Rot_Speed */
void taskInit_PID_Rot_Speed(void *argument)
{
  /* USER CODE BEGIN taskInit_PID_Rot_Speed */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1); // delay 25 ms at the beginning of loop, read delay from macro defined in pid.h
    // here copy feedback position from REG_INPUT_ROT_SPEED to pid struct PV
    // IF PID MODE IS SPEED ONLY, here copy target speed from REG_HOLDING_ROT_TARGET_SPEED to pid struct SP
    // ELSE sp is set in another task, so do nothing :)
    // here call pidUpdate() function
    // here I will copy data from pid struct CV to correct medium (pwm or uart)
  }
  /* USER CODE END taskInit_PID_Rot_Speed */
}

/* USER CODE BEGIN Header_taskInit_PID_Elev_Speed */
/**
 * @brief Function implementing the task_PID_Elev_Speed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskInit_PID_Elev_Speed */
void taskInit_PID_Elev_Speed(void *argument)
{
  /* USER CODE BEGIN taskInit_PID_Elev_Speed */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1); // delay 25 ms at the beginning of loop, read delay from macro defined in pid.h
    // here copy feedback position from REG_INPUT_ELEV_SPEED to pid struct PV
    // IF PID MODE IS SPEED ONLY, here copy target speed from REG_HOLDING_ELEV_TARGET_SPEED to pid struct SP
    // ELSE sp is set in another task, so do nothing :)
    // here call pidUpdate() function
    // here I will copy data from pid struct CV to correct medium (pwm or uart)
  }
  /* USER CODE END taskInit_PID_Elev_Speed */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
