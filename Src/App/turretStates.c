#include "App/turretStates.h"

volatile uint32_t lastFrameReceivedTime = 0;

uint32_t lastUpdateTime = 0;

void checkControlWord(void)
{
    if (regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] & REG_HOLDING_CONTROL_WORD_BIT_ACTIVATE)
    {
        activateTurret();
        regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] &= ~REG_HOLDING_CONTROL_WORD_BIT_ACTIVATE;
    }

    if (regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] & REG_HOLDING_CONTROL_WORD_BIT_FAULT_RESET)
    {
        clearTurretError();
        regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] &= ~REG_HOLDING_CONTROL_WORD_BIT_FAULT_RESET;
    }

    if (regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] & REG_HOLDING_CONTROL_WORD_BIT_RELOAD)
    {
        reloadTurretToggle();
        regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] &= ~REG_HOLDING_CONTROL_WORD_BIT_RELOAD;
    }

    if (regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] & REG_HOLDING_CONTROL_WORD_BIT_SHOOT)
    {
        shootTurretToggle();
        regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] &= ~REG_HOLDING_CONTROL_WORD_BIT_SHOOT;
    }

    if (regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] & REG_HOLDING_CONTROL_WORD_BIT_FLASH_UNLOCK)
    {
        regInput[regInpIx(REG_INPUT_STATUS_WORD)] ^= REG_INPUT_STATUS_WORD_BIT_FLASH_UNLOCKED;
        regHolding[regHoldIx(REG_HOLDING_CONTROL_WORD)] &= ~REG_HOLDING_CONTROL_WORD_BIT_FLASH_UNLOCK;
    }
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

    HAL_GPIO_WritePin(TURRET_LAMP_GPIO_Port, TURRET_LAMP_Pin, GPIO_PIN_SET);
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

    HAL_GPIO_WritePin(TURRET_LAMP_GPIO_Port, TURRET_LAMP_Pin, GPIO_PIN_RESET);
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
    if (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & REG_INPUT_STATUS_WORD_BIT_SHOOTING)
        vTaskResume(task_shootingTimeLimitHandle);
}

void checkIfCommTimeout(void)
{
    if ((HAL_GetTick() - lastFrameReceivedTime > MODBUS_TIMEOUT_ERROR_MS) && (lastFrameReceivedTime != 0))
        setTurretError(REG_INPUT_ERROR_BIT_COMM_TIMEOUT);
}

void stateLEDUpdate(void)
{
    switch (regInput[regInpIx(REG_INPUT_STATUS_WORD)] & (REG_INPUT_STATUS_WORD_BIT_FAULT | REG_INPUT_STATUS_WORD_BIT_ACTIVATED))
    {
    case REG_INPUT_STATUS_WORD_BIT_FAULT:
        HAL_GPIO_TogglePin(BLACKPILL_LED_GPIO_Port, BLACKPILL_LED_Pin);
        break;
    case REG_INPUT_STATUS_WORD_BIT_ACTIVATED:
        HAL_GPIO_WritePin(BLACKPILL_LED_GPIO_Port, BLACKPILL_LED_Pin, GPIO_PIN_RESET);
        break;
    case 0:
        HAL_GPIO_WritePin(BLACKPILL_LED_GPIO_Port, BLACKPILL_LED_Pin, GPIO_PIN_SET);
        break;
    }
}