#ifndef BLUETOOTH_XM_15B_H
#define BLUETOOTH_XM_15B_H

#include "main.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#if USB_BLUETOOTH_DIRECT

extern UART_HandleTypeDef *bt_uart;

void USR_USB_UART_BT_Init(UART_HandleTypeDef *huart);

void USR_UART_BT_RX_Complete(UART_HandleTypeDef *huart);

#endif

#endif