#include "ExternalModules/bluetooth_XM_15B.h"

#if (COMM_MODE & COMM_MODE_BIT_FRWD_USB_BT)

UART_HandleTypeDef *bt_uart;

uint8_t Buffer_APP_BT[COMMS_BUFFER_SIZE];
uint8_t Buffer_BT_APP[1];
uint8_t Buffer_APP_USB[1];

void USR_BT_INIT(UART_HandleTypeDef *huart)
{
    bt_uart = huart;
    HAL_UART_RegisterCallback(bt_uart, HAL_UART_RX_COMPLETE_CB_ID, USR_BT_RX_Complete);
    HAL_UART_Receive_IT(bt_uart, Buffer_BT_APP, 1);
}

void USR_BT_RX_Complete(UART_HandleTypeDef *huart)
{
    strncpy((char *)Buffer_APP_USB, (char *)Buffer_BT_APP, 1);
    HAL_UART_Receive_IT(bt_uart, Buffer_BT_APP, 1);
    CDC_Transmit_FS(Buffer_APP_USB, 1);
}

#endif