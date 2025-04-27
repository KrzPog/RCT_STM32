#include "ModbusRegisters/reg_input.h"

uint8_t lampSwitchCounter = LAMP_SWITCHING_WHEN_FAULT_MS;

uint16_t regInput[REG_INPUT_COUNT] = {0};
volatile uint32_t lastFrameReceivedTime = 0;

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

void activateTurret(void)
{
    //! Turret already activated
    if (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_ACTIVATED)
        return;

    //! Fault bit set, can't activate
    if (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_FAULT)
        return;

    //! Error exist, can't activate
    if (regInput[regInpIx(REG_INPUT_ERROR)])
        return;

    HAL_GPIO_WritePin(TURRET_STATE_LAMP_GPIO_Port, TURRET_STATE_LAMP_Pin, GPIO_PIN_SET);
    regInput[regInpIx(REG_INPUT_STATUS_WORD)] |= REG_INPUT_STATUS_WORD_BIT_ACTIVATED;
}

//! @todo check if turret is in motion, if so, stop it
void deactivateTurret(void)
{
    //! Turret already deactivated
    if (!(regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_ACTIVATED))
        return;

    if (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_RELOADING)
        reloadTurretToggle();

    if (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_SHOOTING)
        shootTurretToggle();

    regInput[regInpIx(REG_INPUT_STATUS_WORD)] &= ~REG_INPUT_STATUS_WORD_BIT_ACTIVATED;
}

void setTurretError(uint16_t errorCode)
{
    regInput[regInpIx(REG_INPUT_STATUS_WORD)] |= REG_INPUT_STATUS_WORD_BIT_FAULT;
    regInput[regInpIx(REG_INPUT_ERROR)] |= errorCode;
    deactivateTurret();
}

//! @todo implement other errors checking
void clearTurretError(void)
{
    regInput[regInpIx(REG_INPUT_ERROR)] &= ~REG_INPUT_ERROR_BIT_COMM_TIMEOUT;

    if (!regInput[regInpIx(REG_INPUT_ERROR)])
        regInput[regInpIx(REG_INPUT_STATUS_WORD)] &= ~REG_INPUT_STATUS_WORD_BIT_FAULT;
}

void reloadTurretToggle(void)
{
    //! Can't start reloading if turret is not activated
    if (!(regInput[regInpIx(REG_INPUT_STATUS_WORD)] & (REG_INPUT_STATUS_WORD_BIT_ACTIVATED | REG_INPUT_STATUS_WORD_BIT_RELOADING)))
        return;

    HAL_GPIO_TogglePin(TURRET_RELOAD_GPIO_Port, TURRET_RELOAD_Pin);
    regInput[regInpIx(REG_INPUT_STATUS_WORD)] ^= REG_INPUT_STATUS_WORD_BIT_RELOADING;
}

void shootTurretToggle(void)
{
    //! Can't start shooting if turret is not activated
    if (!(regInput[regInpIx(REG_INPUT_STATUS_WORD)] & (REG_INPUT_STATUS_WORD_BIT_ACTIVATED | REG_INPUT_STATUS_WORD_BIT_SHOOTING)))
        return;

    HAL_GPIO_TogglePin(TURRET_TRIGGER_GPIO_Port, TURRET_TRIGGER_Pin);
    regInput[regInpIx(REG_INPUT_STATUS_WORD)] ^= REG_INPUT_STATUS_WORD_BIT_SHOOTING;
}

void checkIfCommTimeout(void)
{
    if ((HAL_GetTick() - lastFrameReceivedTime > MODBUS_COMM_TIMEOUT_ERROR_MS) && (lastFrameReceivedTime != 0))
        setTurretError(REG_INPUT_ERROR_BIT_COMM_TIMEOUT);
}

void updateTurretStateLamp(void)
{
    switch (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & (REG_INPUT_STATUS_WORD_BIT_FAULT | REG_INPUT_STATUS_WORD_BIT_ACTIVATED))
    {
    case REG_INPUT_STATUS_WORD_BIT_FAULT:
        if (--lampSwitchCounter == 0)
        {
            lampSwitchCounter = LAMP_SWITCHING_WHEN_FAULT_MS;
            HAL_GPIO_TogglePin(TURRET_STATE_LAMP_GPIO_Port, TURRET_STATE_LAMP_Pin);
        }
        break;
    case REG_INPUT_STATUS_WORD_BIT_ACTIVATED:
        HAL_GPIO_WritePin(TURRET_STATE_LAMP_GPIO_Port, TURRET_STATE_LAMP_Pin, GPIO_PIN_SET);
        break;
    case 0:
        HAL_GPIO_WritePin(TURRET_STATE_LAMP_GPIO_Port, TURRET_STATE_LAMP_Pin, GPIO_PIN_RESET);
        lampSwitchCounter = LAMP_SWITCHING_WHEN_FAULT_MS;
        break;
    }
}