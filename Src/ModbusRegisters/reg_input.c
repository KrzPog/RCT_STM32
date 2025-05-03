#include "ModbusRegisters/reg_input.h"

uint16_t regInput[REG_INPUT_COUNT] = {0};

void initRegInput(void)
{
    regInput[regInpIx(REG_INPUT_COMM_MODE)] = COMM_MODE;

    //! @note turret activated by default, can be changed in the future
    activateTurret();
}

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    //! @note usAddress is 1-based (address specified in request frame + 1)

    lastFrameReceivedTime = HAL_GetTick();

    if ((usAddress - 1) < REG_INPUT_START)
        return MB_ENOREG;

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