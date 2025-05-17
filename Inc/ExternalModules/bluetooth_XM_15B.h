#ifndef BLUETOOTH_XM_15B_H
#define BLUETOOTH_XM_15B_H

#include "main.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#if (COMM_MODE & COMM_MODE_BIT_FRWD_USB_BT)

extern UART_HandleTypeDef *bt_uart;

extern uint8_t Buffer_APP_BT[COMMS_BUFFER_SIZE];

void USR_BT_INIT(UART_HandleTypeDef *huart);
void USR_BT_RX_Complete(UART_HandleTypeDef *huart);

#endif

#endif