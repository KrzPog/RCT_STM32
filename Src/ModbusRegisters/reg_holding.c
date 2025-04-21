#include "ModbusRegisters/reg_holding.h"

uint16_t regHolding[REG_HOLDING_COUNT] = {0x00FF, 0x0FF0, 0xFF00};

void initRegHolding(void)
{
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{

    //! @note usAddress is 1-based (address specified in request frame + 1)

    //! @note Uncomment when REG_HOLDING_START greater than 0
    // if ((usAddress - 1) < REG_HOLDING_START)
    //     return MB_ENOREG;

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
    return MB_ENOERR;
}