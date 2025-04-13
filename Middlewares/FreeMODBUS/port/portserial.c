/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* -----------------------User definitions ----------------------------------*/
UART_HandleTypeDef *hal_uart;
static uint8_t singleChar;

// To implement in case of
#define TRANSMIT
#define RECEIVE

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit(void *serialPort, ULONG ulBaudRate, UCHAR ucDataBits, void *timer)
{
    hal_uart = (UART_HandleTypeDef *)serialPort;
    hal_uart->Init.BaudRate = ulBaudRate;
    hal_uart->Init.WordLength = (ucDataBits == 8) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
    return TRUE;
}

void prvvUARTTxReadyISR(UART_HandleTypeDef *huart)
{
    if (huart->Instance == hal_uart->Instance)
        pxMBFrameCBTransmitterEmpty();
}

void prvvUARTRxISR(UART_HandleTypeDef *huart)
{
    if (huart->Instance == hal_uart->Instance)
        pxMBFrameCBByteReceived();
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    if (xRxEnable)
    {
        RECEIVE;
        HAL_UART_RegisterCallback(hal_uart, HAL_UART_RX_COMPLETE_CB_ID, prvvUARTRxISR);
        HAL_UART_Receive_IT(hal_uart, &singleChar, 1);
    }
    else
    {
        HAL_UART_UnRegisterCallback(hal_uart, HAL_UART_RX_COMPLETE_CB_ID);
        HAL_UART_AbortReceive_IT(hal_uart);
    }
    if (xTxEnable)
    {
        TRANSMIT;
        HAL_UART_RegisterCallback(hal_uart, HAL_UART_TX_COMPLETE_CB_ID, prvvUARTTxReadyISR);
        pxMBFrameCBTransmitterEmpty();
    }
    else
    {
        HAL_UART_UnRegisterCallback(hal_uart, HAL_UART_TX_COMPLETE_CB_ID);
        HAL_UART_AbortTransmit_IT(hal_uart);
    }
}

void vMBPortSerialClose(void)
{
    vMBPortSerialEnable(FALSE, FALSE);
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    HAL_UART_Transmit_IT(hal_uart, (uint8_t *)&ucByte, 1);
    return TRUE;
}

BOOL xMBPortSerialPutBytes(CHAR *ucByte, USHORT usSize)
{
    HAL_UART_Transmit_IT(hal_uart, (uint8_t *)ucByte, usSize);
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR *pucByte)
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
    *pucByte = singleChar;
    HAL_UART_Receive_IT(hal_uart, &singleChar, 1);
    return TRUE;
}