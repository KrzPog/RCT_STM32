#include "App/speedControl.h"

bool elevLimitMinReached = false;
bool elevLimitMaxReached = false;

bool Rot_UART_VESC_Enabled = false;

uint16_t rotControlConfig = 0;
uint16_t elevControlConfig = 0;

int16_t speedCV_rot = 0;
int16_t speedCV_elev = 0;

UART_HandleTypeDef *rot_uart = NULL;
UART_HandleTypeDef *elev_uart = NULL;

static uint8_t rot_rx_char1;
static uint8_t rot_rx_char2;

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
        Rot_UART_VESC_Enabled = true;
        rot_uart = &huart2;
        HAL_UART_Init(rot_uart);
        bldc_interface_uart_init(rot_vesc_send_packet_rot);
        HAL_UART_RegisterCallback(rot_uart, HAL_UART_RX_COMPLETE_CB_ID, rot_vesc_uart_rx_callback);
        HAL_UART_Receive_IT(rot_uart, &rot_rx_char1, 1);
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
        // HAL_UART_Init(&huart6);
        break;
#endif
    }
}

//! @brief Get the current rotation speed
//!
//! @return Current rotation speed in degrees per second (with base 10)

int16_t getRotSpeedCV(void)
{
    uint16_t pidDisabled = regFlash[regFlashIx(REG_FLASH_ROT_CONFIG)] & REG_FLASH_ROT_CONFIG_BIT_PID_EN;
    int16_t speed = 0;
    if (pidDisabled)
        speed = (int16_t)regHolding[regHoldIx(REG_HOLDING_ROT_TARGET_SPEED)];
    else
        speed = (int16_t)PID_speed_rot.values.control_val;

    if (!(regInput[regInpIx(REG_INPUT_STATUS_WORD) & REG_INPUT_STATUS_WORD_BIT_ACTIVATED]))
        speed = 0;

    if (speed == 0)
        return speed;

    if (abs(speed) < regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)])
        speed = (speed > 0) ? regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)] : -regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)];
    else if (abs(speed) > regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)])
        speed = (speed > 0) ? regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)] : -regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)];
    return speed;
}

//! @brief Get the current elevator speed
//!
//! @return Current elevator speed in degrees per second (with base 10)

int16_t getElevSpeedCV(void)
{
    uint16_t pidDisabled = regFlash[regFlashIx(REG_FLASH_ELEV_CONFIG)] & REG_FLASH_ELEV_CONFIG_BIT_PID_EN;
    int16_t speed = 0;
    if (pidDisabled)
        speed = (int16_t)regHolding[regHoldIx(REG_HOLDING_ELEV_TARGET_SPEED)];
    else
        speed = (int16_t)PID_speed_elev.values.control_val;

    if (elevLimitMinReached || elevLimitMaxReached)
        speed = 0;

    if (!(regInput[regInpIx(REG_INPUT_STATUS_WORD) & REG_INPUT_STATUS_WORD_BIT_ACTIVATED]))
        speed = 0;

    if (speed == 0)
        return speed;
    if (abs(speed) < regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)])
        speed = (speed > 0) ? regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)] : -regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)];
    else if (abs(speed) > regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)])
        speed = (speed > 0) ? regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)] : -regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)];
    return speed;
}

void setRotSpeedPWM(int16_t speedCV)
{
    if (rotControlConfig != REG_FLASH_ROT_CONFIG_BITMASK_CONTROL_PWM)
        return;

    uint16_t pwmDuty = (uint32_t)abs(speedCV) * regFlash[regFlashIx(REG_FLASH_ROT_DUTY_PWM_MAX)] / regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)];
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwmDuty);
    if (speedCV > 0)
        HAL_GPIO_WritePin(ROT_DIR_GPIO_Port, ROT_DIR_Pin, GPIO_PIN_SET);
    else if (speedCV < 0)
        HAL_GPIO_WritePin(ROT_DIR_GPIO_Port, ROT_DIR_Pin, GPIO_PIN_RESET);
    USR_Printf_USBD_CDC("Rot\tSpeed: %d, Duty: %d%%, Dir: %c\r\n", speedCV, pwmDuty / 10, (speedCV > 0) ? 'F' : 'B');
}

void setElevSpeedPWM(int16_t speedCV)
{
    if (elevControlConfig != REG_FLASH_ELEV_CONFIG_BITMASK_CONTROL_PWM)
        return;

    uint16_t pwmDuty = (uint32_t)abs(speedCV) * regFlash[regFlashIx(REG_FLASH_ELEV_DUTY_PWM_MAX)] / regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)];
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwmDuty);
    if (speedCV > 0)
        HAL_GPIO_WritePin(ELEV_DIR_GPIO_Port, ELEV_DIR_Pin, GPIO_PIN_SET);
    else if (speedCV < 0)
        HAL_GPIO_WritePin(ELEV_DIR_GPIO_Port, ELEV_DIR_Pin, GPIO_PIN_RESET);
    USR_Printf_USBD_CDC("Elev\tSpeed: %d, Duty: %d%%, Dir: %c\r\n", speedCV, pwmDuty / 10, (speedCV > 0) ? 'F' : 'B');
}

void setRotSpeedUART(int16_t speedCV)
{
    if (rotControlConfig != REG_FLASH_ROT_CONFIG_BITMASK_CONTROL_UART)
        return;

    int16_t uartSpeed = (int32_t)speedCV * regFlash[regFlashIx(REG_FLASH_ROT_UART_SPEED_MAX)] / regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)];
    bldc_interface_set_rpm(uartSpeed);
}

void setElevSpeedUART(int16_t speedCV)
{
#if ELEV_UART_ENABLED
    if (elevControlConfig != REG_FLASH_ELEV_CONFIG_BITMASK_CONTROL_UART)
        return;

    int16_t uartSpeed = (int32_t)speedCV * regFlash[regFlashIx(REG_FLASH_ELEV_UART_SPEED_MAX)] / regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)];
    HAL_UART_Transmit_IT(&huart6, (uint8_t *)&uartSpeed, 2);
#endif
}

void rot_vesc_send_packet_rot(unsigned char *data, unsigned int len)
{
    HAL_UART_Transmit_IT(rot_uart, data, len);
}

void rot_vesc_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart2.Instance)
    {
        rot_rx_char2 = rot_rx_char1;
        bldc_interface_uart_process_byte(rot_rx_char2);
        HAL_UART_Receive_IT(rot_uart, &rot_rx_char1, 1);
    }
}