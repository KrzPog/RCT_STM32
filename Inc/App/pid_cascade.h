#ifndef _PID_CASCADE_H
#define _PID_CASCADE_H

#include "App/pid_speed.h"
#include "App/pid_position.h"
#include <stdbool.h>

#include "ModbusRegisters/reg_holding.h"

// Cascade PID Controller structure
typedef struct {
    MotorID motorId;                    // Motor identifier
    
    // PID Controllers
    PositionPIDController positionPid;  // Outer loop (position)
    PIDController speedPid;             // Inner loop (speed)
    
    // Control mode
    bool positionControlEnabled;        // true = position mode, false = speed mode
    bool isActive;                      // Overall controller active state
    bool pidEnabled;                    // PID enabled/disabled (from flash config)
    
    // Setpoints
    float positionSetpoint;             // Position target [degrees]
    float speedSetpoint;                // Speed target [deg/s]
    
    // Feedback
    float currentPosition;              // Current position [degrees]
    float currentSpeed;                 // Current speed [deg/s]
    
} CascadePIDController;

// Global cascade controllers array
extern CascadePIDController cascadeControllers[MOTOR_COUNT];

// Initialization functions
void cascadePidInit(void);
void cascadePidInitMotor(MotorID motorId);

// Flash configuration loading (similar to speedControl.c)
void cascadePidLoadConfigurationFromFlash(void);

// PID enable/disable check functions
bool cascadePidIsPidEnabled(MotorID motorId);
int16_t cascadePidGetMotorOutput(MotorID motorId);  // Main output function similar to getRotSpeedCV/getElevSpeedCV

// Control mode management
void cascadePidSetControlMode(MotorID motorId, bool positionControl);
bool cascadePidGetControlMode(MotorID motorId);

// Controller activation
void cascadePidSetActive(MotorID motorId, bool active);
bool cascadePidIsActive(MotorID motorId);

// Setpoint management
void cascadePidSetPositionSetpoint(MotorID motorId, float position);
void cascadePidSetSpeedSetpoint(MotorID motorId, float speed);

// Feedback input
void cascadePidUpdateFeedback(MotorID motorId, float position, float speed);

// Main update function
float cascadePidUpdate(MotorID motorId);

// Reset function
void cascadePidReset(MotorID motorId);

// Configuration loading from flash/registers
void cascadePidLoadParametersFromFlash(void);
void cascadePidUpdateFromRegisters(void);

// New functions for individual motor parameter management
void cascadePidUpdatePositionParametersFromFlash(MotorID motorId);
void cascadePidUpdateSpeedParametersFromFlash(MotorID motorId);

// Runtime parameter adjustment
void cascadePidSetPositionParameters(MotorID motorId, float kp, float ki, float kd);
void cascadePidSetSpeedParameters(MotorID motorId, float kp, float ki, float kd);

// Getters for current parameters (useful for debugging/monitoring)
static inline float cascadePidGetPositionKp(MotorID motorId) {
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return cascadeControllers[motorId].positionPid.Kp;
}

static inline float cascadePidGetPositionKi(MotorID motorId) {
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return cascadeControllers[motorId].positionPid.Ki;
}

static inline float cascadePidGetPositionKd(MotorID motorId) {
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return cascadeControllers[motorId].positionPid.Kd;
}

static inline float cascadePidGetSpeedKp(MotorID motorId) {
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return cascadeControllers[motorId].speedPid.Kp;
}

static inline float cascadePidGetSpeedKi(MotorID motorId) {
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return cascadeControllers[motorId].speedPid.Ki;
}

static inline float cascadePidGetSpeedKd(MotorID motorId) {
    if (motorId >= MOTOR_COUNT) return 0.0f;
    return cascadeControllers[motorId].speedPid.Kd;
}

#endif