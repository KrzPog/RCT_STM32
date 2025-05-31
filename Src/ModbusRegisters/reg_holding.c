#include "ModbusRegisters/reg_holding.h"

uint16_t regHolding[REG_HOLDING_COUNT] = {0};
uint16_t regFlash[DATAFLASH_REG_FLASH_SIZE / 2] = {0};

__attribute__((section(".dataflash"), used)) const uint8_t dummy_regFlash[DATAFLASH_REG_FLASH_SIZE]; //!< @note Do not use or modify, dummy variable for linker to see DATAFLASH section

void initRegHolding(void)
{
}

void initRegFlash(void)
{
    uint32_t *data = (uint32_t *)regFlash;
    readDataFlash(0, data, DATAFLASH_REG_FLASH_SIZE / 4);
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

        // Write to flash memory
        uint32_t *data = (uint32_t *)regFlash;
        if (!writeDataFlash(0, data, DATAFLASH_REG_FLASH_SIZE / 4))
            return MB_EILLSTATE; // Flash write failed
    }

    checkControlWord();

    return MB_ENOERR;
}