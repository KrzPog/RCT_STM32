#include "ModbusRegisters/reg_holding.h"

uint16_t regHolding[REG_HOLDING_COUNT];

void initRegHolding(void)
{
    regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] = 0x0000;
    regHolding[regHoldIx(REG_HOLDING_SHOOTING_TIME_LIMIT)] = 0xFFFF; //!< @todo optionally implement
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    //! @note usAddress is 1-based (address specified in request frame + 1)

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

void checkControlWord(void)
{
    if (regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] & REG_HOLDING_CONTROL_WORD_BIT_RELOAD)
    {
        regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] &= ~REG_HOLDING_CONTROL_WORD_BIT_RELOAD;
        HAL_GPIO_TogglePin(TURRET_RELOAD_GPIO_Port, TURRET_RELOAD_Pin);
        regInput[regInpIx(REG_INPUT_STATUS_WORD)] ^= REG_INPUT_STATUS_WORD_BIT_RELOADING;
    }

    if (regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] & REG_HOLDING_CONTROL_WORD_BIT_SHOOT)
    {
        regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] &= ~REG_HOLDING_CONTROL_WORD_BIT_SHOOT;
        HAL_GPIO_TogglePin(TURRET_TRIGGER_GPIO_Port, TURRET_TRIGGER_Pin);
        regInput[regInpIx(REG_INPUT_STATUS_WORD)] ^= REG_INPUT_STATUS_WORD_BIT_SHOOTING;
    }
}