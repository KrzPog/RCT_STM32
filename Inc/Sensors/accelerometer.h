#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>

// Adresy i rejestry MPU6050
#define MPU6050_ADDR           0xD0    // Adres I2C (0x68 << 1)
#define WHO_AM_I_REG           0x75
#define PWR_MGMT_1_REG         0x6B
#define SMPLRT_DIV_REG         0x19
#define ACCEL_CONFIG_REG       0x1C
#define GYRO_CONFIG_REG        0x1B
#define ACCEL_XOUT_H_REG       0x3B
#define GYRO_XOUT_H_REG        0x43

// Funkcje
void MPU6050_Init(void);
void MPU6050_Read_Accel(float* AccelX, float* AccelY, float* AccelZ);
void MPU6050_Read_Gyro(float* GyroX, float* GyroY, float* GyroZ);

#endif /* ACCELEROMETER_H */