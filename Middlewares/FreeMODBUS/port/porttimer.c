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

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* -----------------------User definitions ----------------------------------*/

TIM_HandleTypeDef *hal_tim;
static uint16_t counter = 0; // timer reloads every 50us, but we need 1750us
static uint16_t timeout = 0;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTimeOut50us, void *timer)
{
    hal_tim = (TIM_HandleTypeDef *)timer;
    timeout = usTimeOut50us;
    HAL_TIM_Base_Start_IT(hal_tim);
    return TRUE;
}

void xMBPortTimersClose(void)
{
    HAL_TIM_Base_Stop_IT(hal_tim);
}

void USR_TIM_MB_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (++counter >= timeout)
    {
        counter = 0;
        pxMBPortCBTimerExpired();
    }
}

inline void vMBPortTimersEnable()
{

    HAL_TIM_RegisterCallback(hal_tim, HAL_TIM_PERIOD_ELAPSED_CB_ID, USR_TIM_MB_PeriodElapsedCallback);
}

inline void vMBPortTimersDisable()
{
    HAL_TIM_UnRegisterCallback(hal_tim, HAL_TIM_PERIOD_ELAPSED_CB_ID);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */