#include "ModbusRegisters/reg_holding.h"

uint16_t regHolding[REG_HOLDING_COUNT] = {0};
uint16_t regFlash[REG_FLASH_COUNT] = {0};

void initRegHolding(void)
{
}

void initRegFlash(void)
{
    //! @todo replace it with real flash reading
    regFlash[regFlashIx(REG_FLASH_OPEN_LOOP_MODE)] = 1;       // PID Enabled
    regFlash[regFlashIx(REG_FLASH_ROT_POSITION_MIN)] = -3600; // -360.0 degrees
    regFlash[regFlashIx(REG_FLASH_ROT_POSITION_MAX)] = 3600;  // 360.0 degrees
    regFlash[regFlashIx(REG_FLASH_ELEV_POSITION_MIN)] = -900; // -90.0 degrees
    regFlash[regFlashIx(REG_FLASH_ELEV_POSITION_MAX)] = 900;  // 90.0 degrees
    regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MIN)] = 0;        // 0 degrees per second
    regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)] = 600;      // 60.0 degrees per second
    regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MIN)] = 0;       // 0 degrees per second
    regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)] = 600;     // 60.0 degrees per second
    regFlash[regFlashIx(REG_FLASH_ROT_DUTY_PWM_MIN)] = 0;     // 0 percent
    regFlash[regFlashIx(REG_FLASH_ROT_DUTY_PWM_MAX)] = 1000;  // 100.0 percent
    regFlash[regFlashIx(REG_FLASH_ELEV_DUTY_PWM_MIN)] = 0;    // 0 percent
    regFlash[regFlashIx(REG_FLASH_ELEV_DUTY_PWM_MAX)] = 1000; // 100.0 percent
    regFlash[regFlashIx(REG_FLASH_ROT_UART_SPEED_MIN)] = 0;   // 0 percent
    regFlash[regFlashIx(REG_FLASH_ROT_UART_SPEED_MAX)] = 0xFFFF;
    regFlash[regFlashIx(REG_FLASH_ELEV_UART_SPEED_MIN)] = 0;      // 0 percent
    regFlash[regFlashIx(REG_FLASH_ELEV_UART_SPEED_MAX)] = 0xFFFF; // 100.0 percent
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    //! @note usAddress is 1-based (address specified in request frame + 1)

    lastFrameReceivedTime = HAL_GetTick();

    bool isRegHolding = true;
    isRegHolding &= ((usAddress - 1) >= REG_HOLDING_START);
    isRegHolding &= (((usAddress - 1) + 2 * usNRegs) <= (REG_HOLDING_START + 2 * REG_HOLDING_COUNT));

    bool isRegFlash = true;
    isRegFlash &= ((usAddress - 1) >= REG_FLASH_START);
    isRegFlash &= (((usAddress - 1) + 2 * usNRegs) <= (REG_FLASH_START + 2 * REG_FLASH_COUNT));

    if (!(isRegHolding ^ isRegFlash))
        return MB_ENOREG;

    if ((eMode == MB_REG_READ) && isRegHolding)
    {
        for (int i = 0; i < usNRegs; i++)
        {
            int index = (usAddress - 1 - REG_HOLDING_START) / 2 + i;
            *pucRegBuffer++ = regHolding[index] >> 8;
            *pucRegBuffer++ = regHolding[index] & 0x00FF;
        }
    }

    else if ((eMode == MB_REG_READ) && isRegFlash)
    {
        for (int i = 0; i < usNRegs; i++)
        {
            int index = (usAddress - 1 - REG_FLASH_START) / 2 + i;
            *pucRegBuffer++ = regFlash[index] >> 8;
            *pucRegBuffer++ = regFlash[index] & 0x00FF;
        }
    }

    else if ((eMode == MB_REG_WRITE) && isRegHolding)
    {
        for (int i = 0; i < usNRegs; i++)
        {
            int index = (usAddress - 1 - REG_HOLDING_START) / 2 + i;
            regHolding[index] = (pucRegBuffer[0] << 8) | pucRegBuffer[1];
            pucRegBuffer += 2;
        }
    }

    else if ((eMode == MB_REG_WRITE) && isRegFlash)
    {
        if (!(regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_FLASH_UNLOCKED))
            return MB_EILLSTATE; // Flash access not allowed

        for (int i = 0; i < usNRegs; i++)
        {
            int index = (usAddress - 1 - REG_FLASH_START) / 2 + i;
            regFlash[index] = (pucRegBuffer[0] << 8) | pucRegBuffer[1];
            pucRegBuffer += 2;
        }
    }
    checkControlWord();

    return MB_ENOERR;
}