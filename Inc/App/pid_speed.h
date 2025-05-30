#ifndef _PID_SPEED_H
#define _PID_SPEED_H

#include <stdint.h>
#include <stdbool.h>
#include "pid_output.h"

// Core PID Controller Structure
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
    
    // Output limits
    float outputMin;    // Minimum output value
    float outputMax;    // Maximum output value
    
    // Anti-windup
    float integralMax;  // Maximum integral value
    float integralMin;  // Minimum integral value
    
    // Timing
    uint32_t lastTime;  // Last update time
    float deltaTime;    // Time step in seconds
    
    // Output
    float output;       // PID output
    
} PIDController;

// Function prototypes
void pidInit(PIDController* pid, float kp, float ki, float kd);
void pidSetOutputLimits(PIDController* pid, float min, float max);
void pidSetIntegralLimits(PIDController* pid, float min, float max);
float pidUpdate(PIDController* pid, float setpoint, float feedback, MotorID motorId);
void pidReset(PIDController* pid);

#endif