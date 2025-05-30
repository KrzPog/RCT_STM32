#ifndef _PID_POSITION_H
#define _PID_POSITION_H

#include <stdint.h>
#include <stdbool.h>
#include "pid_output.h"

// Position PID Controller Structure
typedef struct {
    // PID Parameters
    float Kp;           // Proportional gain
    float Ki;           // Integral gain  
    float Kd;           // Derivative gain
    
    // PID State Variables
    float error;        // Current error
    float prevError;    // Previous error
    float integral;     // Integral accumulator
    float derivative;   // Derivative term
    
    // Position limits
    float positionMin;  // Minimum position value
    float positionMax;  // Maximum position value
    
    // Output limits (speed setpoint)
    float outputMin;    // Minimum speed output value
    float outputMax;    // Maximum speed output value
    
    // Anti-windup
    float integralMax;  // Maximum integral value
    float integralMin;  // Minimum integral value
    
    // Timing
    uint32_t lastTime;  // Last update time
    float deltaTime;    // Time step in seconds
    
    // Output (speed setpoint for cascade)
    float output;       // Position PID output (speed setpoint)
    
    // Status
    bool isActive;      // Is position control active
    
} PositionPIDController;

// Function prototypes
void positionPidInit(PositionPIDController* pid, float kp, float ki, float kd, MotorID motorId);
void positionPidSetPositionLimits(PositionPIDController* pid, float min, float max);
void positionPidSetOutputLimits(PositionPIDController* pid, float min, float max);
void positionPidSetIntegralLimits(PositionPIDController* pid, float min, float max);
float positionPidUpdate(PositionPIDController* pid, float setpoint, float feedback, MotorID motorId);
void positionPidReset(PositionPIDController* pid);
void positionPidSetActive(PositionPIDController* pid, bool active);
bool positionPidIsActive(PositionPIDController* pid);

#endif