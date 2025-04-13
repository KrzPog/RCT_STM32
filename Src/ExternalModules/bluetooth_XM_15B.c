#include "ExternalModules/bluetooth_XM_15B.h"

#if USB_BLUETOOTH_DIRECT

UART_HandleTypeDef *bt_uart;

uint8_t Bluetooth_UART_Buffer[USART1_BUFFER_RX + 1];
uint8_t UART_USB_Buffer[USART1_BUFFER_TX + 1];
uint8_t *Bluetooth_UART_Buffer_ptr = &Bluetooth_UART_Buffer[0];

void USR_USB_UART_BT_Init(UART_HandleTypeDef *huart)
{
    bt_uart = huart;
    HAL_UART_RegisterCallback(bt_uart, HAL_UART_RX_COMPLETE_CB_ID, USR_UART_BT_RX_Complete);
    HAL_UART_Receive_IT(bt_uart, Bluetooth_UART_Buffer_ptr, 1);
}

void USR_UART_BT_RX_Complete(UART_HandleTypeDef *huart)
{
    if (*Bluetooth_UART_Buffer_ptr != '\n' && Bluetooth_UART_Buffer_ptr < &Bluetooth_UART_Buffer[USART1_BUFFER_RX])
        HAL_UART_Receive_IT(bt_uart, ++Bluetooth_UART_Buffer_ptr, 1);
    else
    {
        *(++Bluetooth_UART_Buffer_ptr) = '\0';
        strcpy((char *)UART_USB_Buffer, (char *)Bluetooth_UART_Buffer);
        CDC_Transmit_FS(UART_USB_Buffer, Bluetooth_UART_Buffer_ptr - Bluetooth_UART_Buffer + 1);
        Bluetooth_UART_Buffer_ptr = &Bluetooth_UART_Buffer[0];
        HAL_UART_Receive_IT(bt_uart, Bluetooth_UART_Buffer_ptr, 1);
    }
}

#endif