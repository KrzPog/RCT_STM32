#include "ModbusRegisters/reg_holding.h"

uint16_t regHolding[REG_HOLDING_COUNT] = {0};

void initRegHolding(void)
{
    regHolding[regHoldIx(REG_HOLDING_SHOOTING_TIME_LIMIT)] = 0xFFFF; //!< @todo optionally implement
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    //! @note usAddress is 1-based (address specified in request frame + 1)

    lastFrameReceivedTime = HAL_GetTick();

    if ((usAddress - 1) < REG_HOLDING_START)
        return MB_ENOREG;

    if (((usAddress - 1) + 2 * usNRegs) > (REG_HOLDING_START + 2 * REG_HOLDING_COUNT))
        return MB_ENOREG;

    if (eMode == MB_REG_READ)
    {
        for (int i = 0; i < usNRegs; i++)
        {
            int index = (usAddress - 1 - REG_HOLDING_START) / 2 + i;
            *pucRegBuffer++ = regHolding[index] >> 8;
            *pucRegBuffer++ = regHolding[index] & 0x00FF;
        }
    }
    else if (eMode == MB_REG_WRITE)
    {
        for (int i = 0; i < usNRegs; i++)
        {
            int index = (usAddress - 1 - REG_HOLDING_START) / 2 + i;
            regHolding[index] = (pucRegBuffer[0] << 8) | pucRegBuffer[1];
            pucRegBuffer += 2;
        }
    }

    checkControlWord();

    return MB_ENOERR;
}