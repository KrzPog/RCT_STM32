#include "Sensors/accelerometer.h"
#include "ModbusRegisters/reg_input.h"
#include <stdint.h>

/**
 * @brief Inicjalizacja akcelerometru MPU6050
 */
void MPU6050_Init(void)
{
  uint8_t check, data;

  // Sprawdź komunikację
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);

  if (check == 0x68) // 0x68 to domyślny adres MPU6050
  {
    // Power management register - obudzenie urządzenia (wakeup)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);

    // Konfiguracja częstotliwości próbkowania
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);

    // Konfiguracja akcelerometru
    data = 0x00; // Zakres ±2g
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);

    // Konfiguracja żyroskopu
    data = 0x00; // Zakres ±250 deg/s
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);
  }
}

/**
 * @brief Aktualizuje rejestry wejściowe Modbus danymi z akcelerometru i żyroskopu
 */
void accelerometerUpdate(void)
{
  uint8_t data[6];
  int16_t rawX, rawY, rawZ;
  HAL_StatusTypeDef status;

  // Odczyt surowych danych akcelerometru
  status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, 100);
  
  if (status == HAL_OK)
  {
    // Łączenie bajtów
    rawX = (int16_t)((data[0] << 8) | data[1]);
    rawY = (int16_t)((data[2] << 8) | data[3]);
    rawZ = (int16_t)((data[4] << 8) | data[5]);

    // Zapis surowych wartości do rejestrów
    regInput[regInpIx(REG_ACCEL_X)] = (uint16_t)rawX;
    regInput[regInpIx(REG_ACCEL_Y)] = (uint16_t)rawY;
    regInput[regInpIx(REG_ACCEL_Z)] = (uint16_t)rawZ;
  }

  // Odczyt surowych danych żyroskopu
  status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, data, 6, 100);
  
  if (status == HAL_OK)
  {
    // Łączenie bajtów
    rawX = (int16_t)((data[0] << 8) | data[1]);
    rawY = (int16_t)((data[2] << 8) | data[3]);
    rawZ = (int16_t)((data[4] << 8) | data[5]);

    // Zapis surowych wartości do rejestrów
    regInput[regInpIx(REG_GYRO_X)] = (uint16_t)rawX;
    regInput[regInpIx(REG_GYRO_Y)] = (uint16_t)rawY;
    regInput[regInpIx(REG_GYRO_Z)] = (uint16_t)rawZ;
  }
}