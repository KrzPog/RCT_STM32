#include "App/pid_speed.h"
#include "main.h"  // For HAL_GetTick()
#include <string.h>
#include "App/pid_output.h"

void pidInit(PIDController* pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    
    memset(pid, 0, sizeof(PIDController));
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    // Default output limits 
    pid->outputMin = -1000.0f;
    pid->outputMax = 1000.0f;
    
    // Default anti-windup limits
    pid->integralMin = -500.0f;
    pid->integralMax = 500.0f;
    
    pid->deltaTime = 0.01f; // 10ms sampling time
    pid->lastTime = HAL_GetTick();
}

void pidSetOutputLimits(PIDController* pid, float min, float max)
{
    if (pid == NULL) return;
    pid->outputMin = min;
    pid->outputMax = max;
}

void pidSetIntegralLimits(PIDController* pid, float min, float max)
{
    if (pid == NULL) return;
    pid->integralMin = min;
    pid->integralMax = max;
}

float pidUpdate(PIDController* pid, float setpoint, float feedback, MotorID motorId)
{
    if (pid == NULL) return 0.0f;
    
    uint32_t currentTime = HAL_GetTick();
    
    // Skip update if called too frequently
    if (currentTime == pid->lastTime) {
        return pid->output;
    }
    
    // Update delta time
    pid->deltaTime = (currentTime - pid->lastTime) / 1000.0f;
    pid->lastTime = currentTime;
    
    // Calculate error
    pid->error = setpoint - feedback;
    
    // Proportional term
    float proportional = pid->Kp * pid->error;
    
    // Integral term with anti-windup
    pid->integral += pid->error * pid->deltaTime;
    if (pid->integral > pid->integralMax) {
        pid->integral = pid->integralMax;
    } else if (pid->integral < pid->integralMin) {
        pid->integral = pid->integralMin;
    }
    float integral = pid->Ki * pid->integral;
    
    // Derivative term
    pid->derivative = (pid->error - pid->prevError) / pid->deltaTime;
    float derivative = pid->Kd * pid->derivative;
    
    // Calculate total output (before limiting)
    float rawOutput = proportional + integral + derivative;
    
    // Calculate total output (after limiting)
    pid->output = rawOutput;
    
    // Apply output limits
    if (pid->output > pid->outputMax) {
        pid->output = pid->outputMax;
    } else if (pid->output < pid->outputMin) {
        pid->output = pid->outputMin;
    }
    
    // === POPRAWIONA CZĘŚĆ - używamy właściwej funkcji ===
    pidOutputSetSpeed(motorId, pid->output, rawOutput);
    
    // Store current error for next iteration
    pid->prevError = pid->error;
    
    return pid->output;
}

void pidReset(PIDController* pid)
{
    if (pid == NULL) return;
    
    pid->error = 0.0f;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->lastTime = HAL_GetTick();
}