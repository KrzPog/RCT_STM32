#include "App/dataFlash.h"

//! @brief Read data from DATAFLASH Flash sector
//!
//! @param addressOffset Offset in bytes from the start of the DATAFLASH sector
//! @param data Pointer to the buffer where the read data will be stored
//! @param size Number of 32-bit words to read
//!
//! @return true if the read operation was successful, false otherwise
//!
//! @note The DATAFLASH sector is defined by the linker script and should not exceed its bounds.

bool readDataFlash(uint32_t addressOffset, uint32_t *data, uint16_t size)
{
    if (addressOffset % sizeof(uint32_t) != 0)
        return false;

    if (DATAFLASH_START + addressOffset + (size * sizeof(uint32_t)) > DATAFLASH_END)
        return false;

    if (size == 0)
        return false;

    uint32_t *dataAddress = (uint32_t *)(DATAFLASH_START + addressOffset);
    for (uint16_t i = 0; i < size; i++)
        data[i] = dataAddress[i];
    return true;
}

//! @brief Write data to DATAFLASH Flash sector
//!
//! @param addressOffset Offset in bytes from the start of the DATAFLASH sector
//! @param data Pointer to the buffer containing the data to write
//! @param size Number of 32-bit words to write
//!
//! @return true if the write operation was successful, false otherwise
//!
//! @note The DATAFLASH sector is defined by the linker script and should not exceed its bounds.

bool writeDataFlash(uint32_t addressOffset, uint32_t *data, uint16_t size)
{
    if (addressOffset % sizeof(uint32_t) != 0)
        return false;

    if (DATAFLASH_START + addressOffset + (size * sizeof(uint32_t)) > DATAFLASH_END)
        return false;

    if (size == 0)
        return false;

    uint32_t dataAddress = DATAFLASH_START + addressOffset;

    uint32_t sectorError = 0;
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInitStructure = {0};
    eraseInitStructure.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStructure.Sector = FLASH_SECTOR_7;
    eraseInitStructure.NbSectors = 1;
    eraseInitStructure.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();

    status = HAL_FLASHEx_Erase(&eraseInitStructure, &sectorError);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return false;
    }

    for (int i = 0; i < size; i++, dataAddress += sizeof(uint32_t))
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dataAddress, data[i]);
        if (status != HAL_OK)
        {
            uint32_t error_code = HAL_FLASH_GetError();
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}