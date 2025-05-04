#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "main.h"
#include "i2c.h"

// Adres MPU6050 (AD0 podłączony do GND)
#define MPU6050_ADDR 0xD0

// Rejestry MPU6050
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define ACCEL_CONFIG_REG    0x1C
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43

// Deklaracje funkcji
void MPU6050_Init(void);
void MPU6050_Read_Accel(float* AccelX, float* AccelY, float* AccelZ);
void MPU6050_Read_Gyro(float* GyroX, float* GyroY, float* GyroZ);

/**
 * @brief Aktualizuj rejestry wejściowe Modbus danymi z akcelerometru i żyroskopu
 */
void accelerometerUpdate(void);

#endif /* ACCELEROMETER_H */