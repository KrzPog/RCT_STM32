#include "ModbusRegisters/reg_input.h"

uint16_t regInput[REG_INPUT_COUNT] = {0x0000, 0x000F, 0x00F0, 0x0F00, 0xF000};

void initRegInput(void)
{
    regInput[REG_INPUT_COMM_MODE] = COMM_MODE;
}

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    //! @note usAddress is 1-based (address specified in request frame + 1)

    //! @note Uncomment when REG_INPUT_START greater than 0
    // if ((usAddress - 1) < REG_INPUT_START)
    //     return MB_ENOREG;

    if (((usAddress - 1) + 2 * usNRegs) > (REG_INPUT_START + 2 * REG_INPUT_COUNT))
        return MB_ENOREG;

    for (int i = 0; i < usNRegs; i++)
    {
        int index = (usAddress - 1 - REG_INPUT_START) / 2 + i;
        *pucRegBuffer++ = (UCHAR)(regInput[index] >> 8);
        *pucRegBuffer++ = (UCHAR)(regInput[index] & 0x00FF);
    }
    return MB_ENOERR;
}