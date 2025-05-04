#include "Sensors/accelerometer.h"
#include "ModbusRegisters/reg_input.h"
#include <stdint.h>

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

void MPU6050_Read_Accel(float *AccelX, float *AccelY, float *AccelZ)
{
  uint8_t data[6];
  int16_t rawX, rawY, rawZ;

  // Odczyt 6 bajtów danych począwszy od rejestru ACCEL_XOUT_H
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, 100);

  // Łączenie bajtów
  rawX = (int16_t)((data[0] << 8) | data[1]);
  rawY = (int16_t)((data[2] << 8) | data[3]);
  rawZ = (int16_t)((data[4] << 8) | data[5]);

  // Konwersja na jednostki g (przy zakresie ±2g)
  *AccelX = rawX / 16384.0f;
  *AccelY = rawY / 16384.0f;
  *AccelZ = rawZ / 16384.0f;
}

void MPU6050_Read_Gyro(float *GyroX, float *GyroY, float *GyroZ)
{
  uint8_t data[6];
  int16_t rawX, rawY, rawZ;

  // Odczyt 6 bajtów danych począwszy od rejestru GYRO_XOUT_H
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, data, 6, 100);

  // Łączenie bajtów
  rawX = (int16_t)((data[0] << 8) | data[1]);
  rawY = (int16_t)((data[2] << 8) | data[3]);
  rawZ = (int16_t)((data[4] << 8) | data[5]);

  // Konwersja na deg/s (przy zakresie ±250 deg/s)
  *GyroX = rawX / 131.0f;
  *GyroY = rawY / 131.0f;
  *GyroZ = rawZ / 131.0f;
}

/**
 * @brief Aktualizuj rejestry wejściowe Modbus danymi z akcelerometru i żyroskopu
 *
 * Ta funkcja czyta dane z MPU6050 i zapisuje je do rejestrów wejściowych.
 * Ponieważ w reg_input.h nie ma zdefiniowanych rejestrów dla tych danych,
 * używamy zdefiniowanych poniżej adresów. Upewnij się, że są one wolne
 * i nie kolidują z innymi rejestrami.
 */
void accelerometerUpdate(void)
{
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;

  // Odczyt danych z MPU6050
  MPU6050_Read_Accel(&accelX, &accelY, &accelZ);
  MPU6050_Read_Gyro(&gyroX, &gyroY, &gyroZ);

  // Przygotowanie wartości dla rejestrów Modbus
  // Dla akcelerometru: przeliczamy wartości z zakresu ±2g na int16_t (-32768 do 32767)
  // Wartość 1g = 16384 jednostek, czyli pełna skala to ok. ±2g
  int16_t accelX_int = (int16_t)(accelX * 16384.0f);
  int16_t accelY_int = (int16_t)(accelY * 16384.0f);
  int16_t accelZ_int = (int16_t)(accelZ * 16384.0f);

  // Dla żyroskopu: przeliczamy wartości z zakresu ±250°/s na int16_t (-32768 do 32767)
  // Wartość 250°/s = 32768 jednostek, czyli pełna skala to ok. ±250°/s
  int16_t gyroX_int = (int16_t)(gyroX * 131.0f);
  int16_t gyroY_int = (int16_t)(gyroY * 131.0f);
  int16_t gyroZ_int = (int16_t)(gyroZ * 131.0f);

  regInput[regInpIx(REG_ACCEL_X)] = (uint16_t)accelX_int;
  regInput[regInpIx(REG_ACCEL_Y)] = (uint16_t)accelY_int;
  regInput[regInpIx(REG_ACCEL_Z)] = (uint16_t)accelZ_int;
  regInput[regInpIx(REG_GYRO_X)] = (uint16_t)gyroX_int;
  regInput[regInpIx(REG_GYRO_Y)] = (uint16_t)gyroY_int;
  regInput[regInpIx(REG_GYRO_Z)] = (uint16_t)gyroZ_int;
}