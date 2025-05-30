#include "App/pid_position.h"
#include "main.h"  // For HAL_GetTick()
#include <string.h>
#include "ModbusRegisters/reg_holding.h"

void positionPidInit(PositionPIDController* pid, float kp, float ki, float kd, MotorID motorId)
{
    if (pid == NULL) return;
    
    memset(pid, 0, sizeof(PositionPIDController));
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    // Set position limits based on motor type from flash registers
    if (motorId == MOTOR_ROTATION) {
        pid->positionMin = (float)((int16_t)regFlash[regFlashIx(REG_FLASH_ROT_POSITION_MIN)]) / 10.0f;
        pid->positionMax = (float)((int16_t)regFlash[regFlashIx(REG_FLASH_ROT_POSITION_MAX)]) / 10.0f;
        
        // Speed output limits from flash
        pid->outputMin = -(float)regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)] / 10.0f;
        pid->outputMax = (float)regFlash[regFlashIx(REG_FLASH_ROT_SPEED_MAX)] / 10.0f;
    } else if (motorId == MOTOR_ELEVATION) {
        pid->positionMin = (float)((int16_t)regFlash[regFlashIx(REG_FLASH_ELEV_POSITION_MIN)]) / 10.0f;
        pid->positionMax = (float)((int16_t)regFlash[regFlashIx(REG_FLASH_ELEV_POSITION_MAX)]) / 10.0f;
        
        // Speed output limits from flash
        pid->outputMin = -(float)regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)] / 10.0f;
        pid->outputMax = (float)regFlash[regFlashIx(REG_FLASH_ELEV_SPEED_MAX)] / 10.0f;
    }
    
    // Default anti-windup limits (can be adjusted)
    pid->integralMin = -100.0f;
    pid->integralMax = 100.0f;
    
    pid->deltaTime = 0.02f; // 20ms sampling time (slower than speed loop)
    pid->lastTime = HAL_GetTick();
    pid->isActive = false;
}

void positionPidSetPositionLimits(PositionPIDController* pid, float min, float max)
{
    if (pid == NULL) return;
    pid->positionMin = min;
    pid->positionMax = max;
}

void positionPidSetOutputLimits(PositionPIDController* pid, float min, float max)
{
    if (pid == NULL) return;
    pid->outputMin = min;
    pid->outputMax = max;
}

void positionPidSetIntegralLimits(PositionPIDController* pid, float min, float max)
{
    if (pid == NULL) return;
    pid->integralMin = min;
    pid->integralMax = max;
}

float positionPidUpdate(PositionPIDController* pid, float setpoint, float feedback, MotorID motorId)
{
    if (pid == NULL || !pid->isActive) return 0.0f;
    
    uint32_t currentTime = HAL_GetTick();
    
    // Skip update if called too frequently (position loop should be slower)
    if ((currentTime - pid->lastTime) < 20) { // Minimum 20ms between updates
        return pid->output;
    }
    
    // Update delta time
    pid->deltaTime = (currentTime - pid->lastTime) / 1000.0f;
    pid->lastTime = currentTime;
    
    // Clamp setpoint to position limits
    if (setpoint > pid->positionMax) {
        setpoint = pid->positionMax;
    } else if (setpoint < pid->positionMin) {
        setpoint = pid->positionMin;
    }
    
    // Calculate error
    pid->error = setpoint - feedback;
    
    // For rotation, handle wrap-around (shortest path)
    if (motorId == MOTOR_ROTATION) {
        // Handle 360-degree wrap-around
        if (pid->error > 180.0f) {
            pid->error -= 360.0f;
        } else if (pid->error < -180.0f) {
            pid->error += 360.0f;
        }
    }
    
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
    
    // Calculate total output (speed setpoint)
    pid->output = proportional + integral + derivative;
    
    // Apply output limits (speed limits)
    if (pid->output > pid->outputMax) {
        pid->output = pid->outputMax;
    } else if (pid->output < pid->outputMin) {
        pid->output = pid->outputMin;
    }
    
    // Store current error for next iteration
    pid->prevError = pid->error;
    
    return pid->output;
}

void positionPidReset(PositionPIDController* pid)
{
    if (pid == NULL) return;
    
    pid->error = 0.0f;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->lastTime = HAL_GetTick();
}

void positionPidSetActive(PositionPIDController* pid, bool active)
{
    if (pid == NULL) return;
    
    pid->isActive = active;
    
    // Reset PID when activating/deactivating
    if (active) {
        positionPidReset(pid);
    } else {
        pid->output = 0.0f;
    }
}

bool positionPidIsActive(PositionPIDController* pid)
{
    if (pid == NULL) return false;
    return pid->isActive;
}