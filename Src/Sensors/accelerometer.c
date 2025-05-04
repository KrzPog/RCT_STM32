#include "Sensors/accelerometer.h"
#include <stdint.h>

void MPU6050_Init(void)
{
  uint8_t check, data;
  
  // Sprawdź komunikację
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
  
  if (check == 0x68)  // 0x68 to domyślny adres MPU6050
  {
    // Power management register - obudzenie urządzenia (wakeup)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
    
    // Konfiguracja częstotliwości próbkowania
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);
    
    // Konfiguracja akcelerometru
    data = 0x00;  // Zakres ±2g
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);
    
    // Konfiguracja żyroskopu
    data = 0x00;  // Zakres ±250 deg/s
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);
  }
}

void MPU6050_Read_Accel(float* AccelX, float* AccelY, float* AccelZ)
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

void MPU6050_Read_Gyro(float* GyroX, float* GyroY, float* GyroZ)
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