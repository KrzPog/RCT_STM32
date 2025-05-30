#ifndef _PID_CASCADE_H
#define _PID_CASCADE_H

#include <stdint.h>
#include <stdbool.h>
#include "pid_position.h"
#include "pid_speed.h"
#include "pid_output.h"

// Cascade PID Controller Structure
typedef struct {
    PositionPIDController positionPid;  // Position controller (outer loop)
    PIDController speedPid;             // Speed controller (inner loop)
    
    MotorID motorId;                    // Motor identifier
    
    // Control mode
    bool positionControlEnabled;        // True for position control, false for speed only
    bool isActive;                      // Is cascade controller active
    
    // Feedback inputs
    float currentPosition;              // Current position feedback
    float currentSpeed;                 // Current speed feedback
    
    // Setpoints
    float positionSetpoint;             // Desired position
    float speedSetpoint;                // Desired speed (direct or from position PID)
    
} CascadePIDController;

// Global cascade controllers
extern CascadePIDController cascadeControllers[MOTOR_COUNT];

// Function prototypes
void cascadePidInit(void);
void cascadePidInitMotor(MotorID motorId);
void cascadePidSetControlMode(MotorID motorId, bool positionControl);
bool cascadePidGetControlMode(MotorID motorId);
void cascadePidSetActive(MotorID motorId, bool active);
bool cascadePidIsActive(MotorID motorId);

void cascadePidSetPositionSetpoint(MotorID motorId, float position);
void cascadePidSetSpeedSetpoint(MotorID motorId, float speed);
void cascadePidUpdateFeedback(MotorID motorId, float position, float speed);

float cascadePidUpdate(MotorID motorId);
void cascadePidReset(MotorID motorId);

// Utility functions
void cascadePidLoadParametersFromFlash(void);
void cascadePidUpdateFromRegisters(void);

#endif