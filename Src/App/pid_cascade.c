#include "App/pid_cascade.h"
#include "App/pid_parameters.h"
#include "ModbusRegisters/reg_holding.h"
#include "main.h"
#include <string.h>

// Global cascade controllers - now truly independent
CascadePIDController cascadeControllers[MOTOR_COUNT];

// Local configuration variables - similar to speedControl.c
uint16_t rotPidConfig = 0;
uint16_t elevPidConfig = 0;
bool rotPidEnabled = false;
bool elevPidEnabled = false;
bool rotPositionControlEnabled = false;
bool elevPositionControlEnabled = false;

void cascadePidInit(void)
{
    // Load configuration from flash registers first
    cascadePidLoadConfigurationFromFlash();
    
    // Initialize all cascade controllers
    for (MotorID motorId = 0; motorId < MOTOR_COUNT; motorId++) {
        cascadePidInitMotor(motorId);
    }
}

void cascadePidLoadConfigurationFromFlash(void)
{
    // Load rotation motor configuration
    rotPidConfig = regFlash[regFlashIx(REG_FLASH_ROT_CONFIG)];
    rotPidEnabled = !(rotPidConfig & REG_FLASH_ROG_CONFIG_BIT_PID_EN);  // 0 = PID enabled
    rotPositionControlEnabled = (rotPidConfig & REG_FLASH_ROT_CONFIG_BIT_PID_TYPE) != 0;  // 1 = Position control
    
    // Load elevation motor configuration
    elevPidConfig = regFlash[regFlashIx(REG_FLASH_ELEV_CONFIG)];
    elevPidEnabled = !(elevPidConfig & REG_FLASH_ELEV_CONFIG_BIT_PID_EN);  // 0 = PID enabled
    elevPositionControlEnabled = (elevPidConfig & REG_FLASH_ELEV_CONFIG_BIT_PID_TYPE) != 0;  // 1 = Position control
}

void cascadePidInitMotor(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // Clear structure
    memset(cascade, 0, sizeof(CascadePIDController));
    cascade->motorId = motorId;
    
    // Initialize position PID (outer loop) with motor-specific parameters
    const MotorPIDParams* posParams = &MOTOR_POSITION_PID_PARAMS[motorId];
    positionPidInit(&cascade->positionPid, 
                   posParams->Kp, 
                   posParams->Ki, 
                   posParams->Kd, 
                   motorId);
    positionPidSetIntegralLimits(&cascade->positionPid, 
                                posParams->integralMin, 
                                posParams->integralMax);
    
    // Initialize speed PID (inner loop) with motor-specific parameters
    const MotorPIDParams* speedParams = &MOTOR_SPEED_PID_PARAMS[motorId];
    pidInit(&cascade->speedPid, 
            speedParams->Kp, 
            speedParams->Ki, 
            speedParams->Kd);
    pidSetIntegralLimits(&cascade->speedPid, 
                        speedParams->integralMin, 
                        speedParams->integralMax);
    
    // Set control mode based on flash configuration
    bool positionControl = false;
    bool pidEnabled = false;
    
    if (motorId == MOTOR_ROTATION) {
        positionControl = rotPositionControlEnabled;
        pidEnabled = rotPidEnabled;
    } else if (motorId == MOTOR_ELEVATION) {
        positionControl = elevPositionControlEnabled;
        pidEnabled = elevPidEnabled;
    }
    
    cascade->positionControlEnabled = positionControl;
    cascade->isActive = false;
    cascade->pidEnabled = pidEnabled;
    
    // Initialize PID output structure
    pidOutputSetControlMode(motorId, positionControl);
}

bool cascadePidIsPidEnabled(MotorID motorId)
{
    if (motorId == MOTOR_ROTATION) {
        return rotPidEnabled;
    } else if (motorId == MOTOR_ELEVATION) {
        return elevPidEnabled;
    }
    return false;
}

int16_t cascadePidGetMotorOutput(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // Check if PID is enabled for this motor (similar to speedControl.c)
    bool pidEnabled = cascadePidIsPidEnabled(motorId);
    
    if (!pidEnabled) {
        // PID disabled - return direct setpoint (open loop control)
        if (cascade->positionControlEnabled) {
            // In position mode but PID disabled - this is unusual, return 0
            return 0;
        } else {
            // In speed mode and PID disabled - return speed setpoint directly
            // Convert from float degrees/sec to int16 with base 10
            return (int16_t)(cascade->speedSetpoint * 10.0f);
        }
    } else {
        // PID enabled - return PID output
        if (!cascade->isActive) {
            return 0;
        }
        
        float pidOutput = cascadePidUpdate(motorId);
        // Convert from float to int16 with appropriate scaling
        return (int16_t)(pidOutput * 10.0f);
    }
}

void cascadePidSetControlMode(MotorID motorId, bool positionControl)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // If switching modes, reset both controllers
    if (cascade->positionControlEnabled != positionControl) {
        cascadePidReset(motorId);
    }
    
    cascade->positionControlEnabled = positionControl;
    
    // Set active state for position controller
    positionPidSetActive(&cascade->positionPid, positionControl);
    
    // Update PID output structure
    pidOutputSetControlMode(motorId, positionControl);
    pidOutputSetPositionActive(motorId, positionControl);
}

bool cascadePidGetControlMode(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return cascadeControllers[motorId].positionControlEnabled;
}

void cascadePidSetActive(MotorID motorId, bool active)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    cascade->isActive = active;
    
    // Set PID output active state
    pidOutputSetActive(motorId, active);
    
    // Set position PID active if in position control mode
    if (cascade->positionControlEnabled) {
        positionPidSetActive(&cascade->positionPid, active);
        pidOutputSetPositionActive(motorId, active);
    }
    
    // Reset controllers when activating
    if (active) {
        cascadePidReset(motorId);
    }
}

bool cascadePidIsActive(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return false;
    return cascadeControllers[motorId].isActive;
}

void cascadePidSetPositionSetpoint(MotorID motorId, float position)
{
    if (motorId >= MOTOR_COUNT) return;
    cascadeControllers[motorId].positionSetpoint = position;
    
    // Update PID output structure
    pidOutputSetSetpoints(motorId, position, cascadeControllers[motorId].speedSetpoint);
}

void cascadePidSetSpeedSetpoint(MotorID motorId, float speed)
{
    if (motorId >= MOTOR_COUNT) return;
    cascadeControllers[motorId].speedSetpoint = speed;
    
    // Update PID output structure
    pidOutputSetSetpoints(motorId, cascadeControllers[motorId].positionSetpoint, speed);
}

void cascadePidUpdateFeedback(MotorID motorId, float position, float speed)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    cascade->currentPosition = position;
    cascade->currentSpeed = speed;
    
    // Update PID output structure with feedback
    pidOutputSetFeedback(motorId, position, speed);
}

float cascadePidUpdate(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return 0.0f;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    if (!cascade->isActive) {
        return 0.0f;
    }
    
    // Check if PID is enabled for this motor
    if (!cascadePidIsPidEnabled(motorId)) {
        // PID disabled - return 0 (open loop control handled elsewhere)
        return 0.0f;
    }
    
    float finalSpeedSetpoint;
    
    if (cascade->positionControlEnabled) {
        // Position control mode - cascade operation
        // Outer loop: Position PID generates speed setpoint
        finalSpeedSetpoint = positionPidUpdate(&cascade->positionPid,
                                             cascade->positionSetpoint,
                                             cascade->currentPosition,
                                             motorId);
        
        // Update position output in PID output structure
        pidOutputSetPosition(motorId, finalSpeedSetpoint, cascade->positionPid.error);
    } else {
        // Speed-only control mode
        finalSpeedSetpoint = cascade->speedSetpoint;
    }
    
    // Inner loop: Speed PID generates motor output
    float motorOutput = pidUpdate(&cascade->speedPid,
                                 finalSpeedSetpoint,
                                 cascade->currentSpeed,
                                 motorId);
    
    // Set final output in PID output structure
    pidOutputSetFinal(motorId, motorOutput);
    
    return motorOutput;
}

void cascadePidReset(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // Reset both controllers
    positionPidReset(&cascade->positionPid);
    pidReset(&cascade->speedPid);
    
    // Clear setpoints
    cascade->positionSetpoint = cascade->currentPosition;
    cascade->speedSetpoint = 0.0f;
    
    // Update PID output structure
    pidOutputSetSetpoints(motorId, cascade->positionSetpoint, cascade->speedSetpoint);
}

void cascadePidLoadParametersFromFlash(void)
{
    // Reload configuration from flash
    cascadePidLoadConfigurationFromFlash();
    
    // Update control modes from flash - now motor-specific
    for (MotorID motorId = 0; motorId < MOTOR_COUNT; motorId++) {
        bool positionControl = false;
        
        if (motorId == MOTOR_ROTATION) {
            positionControl = rotPositionControlEnabled;
        } else if (motorId == MOTOR_ELEVATION) {
            positionControl = elevPositionControlEnabled;
        }
        
        cascadePidSetControlMode(motorId, positionControl);
    }
}

void cascadePidUpdateFromRegisters(void)
{
    // Update setpoints from Modbus registers
    
    // Position setpoints (in degrees with base 10)
    float rotPositionSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ROT_TARGET_POSITION)]) / 10.0f;
    float elevPositionSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ELEV_TARGET_POSITION)]) / 10.0f;
    
    cascadePidSetPositionSetpoint(MOTOR_ROTATION, rotPositionSetpoint);
    cascadePidSetPositionSetpoint(MOTOR_ELEVATION, elevPositionSetpoint);
    
    // Speed setpoints (in degrees per second with base 10)
    float rotSpeedSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ROT_TARGET_SPEED)]) / 10.0f;
    float elevSpeedSetpoint = (float)((int16_t)regHolding[regHoldIx(REG_HOLDING_ELEV_TARGET_SPEED)]) / 10.0f;
    
    cascadePidSetSpeedSetpoint(MOTOR_ROTATION, rotSpeedSetpoint);
    cascadePidSetSpeedSetpoint(MOTOR_ELEVATION, elevSpeedSetpoint);
    
    // Update control modes if changed
    cascadePidLoadParametersFromFlash();
}

// New functions for individual motor parameter management

void cascadePidUpdatePositionParametersFromFlash(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // Reload position PID parameters from flash/EEPROM if needed
    // This function can be extended to read parameters from flash registers
    const MotorPIDParams* posParams = &MOTOR_POSITION_PID_PARAMS[motorId];
    
    cascade->positionPid.Kp = posParams->Kp;
    cascade->positionPid.Ki = posParams->Ki;
    cascade->positionPid.Kd = posParams->Kd;
    
    positionPidSetIntegralLimits(&cascade->positionPid, 
                                posParams->integralMin, 
                                posParams->integralMax);
}

void cascadePidUpdateSpeedParametersFromFlash(MotorID motorId)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    // Reload speed PID parameters from flash/EEPROM if needed
    const MotorPIDParams* speedParams = &MOTOR_SPEED_PID_PARAMS[motorId];
    
    cascade->speedPid.Kp = speedParams->Kp;
    cascade->speedPid.Ki = speedParams->Ki;
    cascade->speedPid.Kd = speedParams->Kd;
    
    pidSetIntegralLimits(&cascade->speedPid, 
                        speedParams->integralMin, 
                        speedParams->integralMax);
}

void cascadePidSetPositionParameters(MotorID motorId, float kp, float ki, float kd)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    cascade->positionPid.Kp = kp;
    cascade->positionPid.Ki = ki;
    cascade->positionPid.Kd = kd;
    
    // Reset controller after parameter change
    positionPidReset(&cascade->positionPid);
}

void cascadePidSetSpeedParameters(MotorID motorId, float kp, float ki, float kd)
{
    if (motorId >= MOTOR_COUNT) return;
    
    CascadePIDController* cascade = &cascadeControllers[motorId];
    
    cascade->speedPid.Kp = kp;
    cascade->speedPid.Ki = ki;
    cascade->speedPid.Kd = kd;
    
    // Reset controller after parameter change
    pidReset(&cascade->speedPid);
}