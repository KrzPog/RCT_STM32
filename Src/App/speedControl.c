#include "App/speedControl.h"

bool elevLimitMinReached = false;
bool elevLimitMaxReached = false;

uint16_t rotControlConfig = 0;
uint16_t elevControlConfig = 0;

void initRotSpeedControl(void)
{
    rotControlConfig = regFlash[regFlashIx(REG_FLASH_ROT_CONFIG)] & REG_FLASH_ROT_CONFIG_BITMASK_CONTROL;
    switch (rotControlConfig)
    {
    case REG_FLASH_ROT_CONFIG_BITMASK_CONTROL_PWM:
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
        break;
    case REG_FLASH_ROT_CONFIG_BITMASK_CONTROL_UART:
        HAL_UART_Init(&huart2);
        break;
    }
}

void initElevSpeedControl(void)
{
    elevControlConfig = regFlash[regFlashIx(REG_FLASH_ELEV_CONFIG)] & REG_FLASH_ELEV_CONFIG_BITMASK_CONTROL;
    switch (elevControlConfig)
    {
    case REG_FLASH_ELEV_CONFIG_BITMASK_CONTROL_PWM:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
        break;
#if ELEV_UART_ENABLED
    case REG_FLASH_ELEV_CONFIG_BITMASK_CONTROL_UART:
        HAL_UART_Init(&huart6);
        break;
#endif
    }
}

int16_t getRotSpeedCV(void)
{
    uint16_t pidDisabled = regFlash[regFlashIx(REG_FLASH_ROT_CONFIG)] & REG_FLASH_ROG_CONFIG_BIT_PID_EN;
    if (pidDisabled)
        return (int16_t)regHolding[regHoldIx(REG_HOLDING_ROT_TARGET_SPEED)];
    else
        return (int16_t)PID_speed_rot.values.control_val;
}

int16_t getElevSpeedCV(void)
{
    uint16_t pidDisabled = regFlash[regFlashIx(REG_FLASH_ELEV_CONFIG)] & REG_FLASH_ELEV_CONFIG_BIT_PID_EN;
    if (pidDisabled)
        return (int16_t)regHolding[regHoldIx(REG_HOLDING_ELEV_TARGET_SPEED)];
    else
        return (int16_t)PID_speed_elev.values.control_val;
}

void sendRotSpeedCV(void)
{
    int16_t speedCV = getRotSpeedCV();
    if (speedCV > 0 && speedCV < regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)])
        speedCV = regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)];
    else if (speedCV < 0 && speedCV > -regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)])
        speedCV = -regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)];
    else if (speedCV > regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)])
        speedCV = regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)];
    else if (speedCV < -regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)])
        speedCV = -regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)];

    if (rotControlConfig == REG_FLASH_ROT_CONFIG_BITMASK_CONTROL_PWM)
    {
        uint16_t pwmDuty = (uint32_t)abs(speedCV) * regFlash[regFlashIx(REG_FLASH_ROT_DUTY_PWM_MAX)] / regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)];
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwmDuty);
        if (speedCV > 0)
            HAL_GPIO_WritePin(ROT_DIR_GPIO_Port, ROT_DIR_Pin, GPIO_PIN_SET);
        else if (speedCV < 0)
            HAL_GPIO_WritePin(ROT_DIR_GPIO_Port, ROT_DIR_Pin, GPIO_PIN_RESET);
    }

    else if (rotControlConfig == REG_FLASH_ROT_CONFIG_BITMASK_CONTROL_UART)
    {
        uint16_t uartSpeed = (uint32_t)speedCV * regFlash[regFlashIx(REG_FLASH_ROT_UART_SPEED_MAX)] / regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)];
        uint8_t data[2] = {speedCV >> 8, speedCV & 0xFF};
        HAL_UART_Transmit_IT(&huart2, data, sizeof(data));
    }
}

void sendElevSpeedCV(void)
{
    int16_t speedCV = getElevSpeedCV();
    if (elevLimitMinReached || elevLimitMaxReached)
    {
        speedCV = 0; // Stop the elevator if limits are reached
    }
    else
    {
        if (speedCV > 0 && speedCV < regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)])
            speedCV = regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)];
        else if (speedCV < 0 && speedCV > -regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)])
            speedCV = -regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)];
        else if (speedCV > regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)])
            speedCV = regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)];
        else if (speedCV < -regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)])
            speedCV = -regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)];
    }

    if (elevControlConfig == REG_FLASH_ELEV_CONFIG_BITMASK_CONTROL_PWM)
    {
        uint16_t pwmDuty = (uint32_t)abs(speedCV) * regFlash[regFlashIx(REG_FLASH_ELEV_DUTY_PWM_MAX)] / regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)];
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwmDuty);
        if (speedCV > 0)
            HAL_GPIO_WritePin(ELEV_DIR_GPIO_Port, ELEV_DIR_Pin, GPIO_PIN_SET);
        else if (speedCV < 0)
            HAL_GPIO_WritePin(ELEV_DIR_GPIO_Port, ELEV_DIR_Pin, GPIO_PIN_RESET);
    }

#if ELEV_UART_ENABLED
    else if (elevControlConfig == REG_FLASH_ELEV_CONFIG_BITMASK_CONTROL_UART)
    {
        uint16_t uartSpeed = (uint32_t)speedCV * regFlash[regFlashIx(REG_FLASH_ELEV_UART_SPEED_MAX)] / regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)];
        uint8_t data[2] = {speedCV >> 8, speedCV & 0xFF};
        HAL_UART_Transmit_IT(&huart6, data, sizeof(data));
    }
#endif
}