#include "App/pid.h"

// External PID controller instances
PID PID_speed_elev;
PID PID_position_elev;
PID PID_speed_rot;
PID PID_position_rot;

/**
 * @brief Initialize PID controller
 * @param pPID Pointer to PID structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param sampling_time_ms Sampling time in milliseconds
 */
void PID_Init(PID *pPID, float kp, float ki, float kd, uint16_t sampling_time_ms)
{
    if (pPID == NULL)
        return;
    
    // Set PID parameters
    pPID->parameters.Kp = kp;
    pPID->parameters.Ki = ki;
    pPID->parameters.Kd = kd;
    
    // Initialize values
    pPID->values.setpoint = 0.0f;
    pPID->values.current_val = 0.0f;
    pPID->values.control_val = 0.0f;
    
    // Initialize internal variables
    pPID->integral = 0.0f;
    pPID->prev_error = 0.0f;
    pPID->first_run = true;
    pPID->prevTime = HAL_GetTick();
    pPID->sampling_time_ms = sampling_time_ms;
    
    // Initialize output limits (disabled by default)
    pPID->output_min = 0.0f;
    pPID->output_max = 0.0f;
    pPID->limits_enabled = false;
}

/**
 * @brief Set output limits for PID controller
 * @param pPID Pointer to PID structure
 * @param min Minimum output value
 * @param max Maximum output value
 */
void PID_SetLimits(PID *pPID, float min, float max)
{
    if (pPID == NULL)
        return;
    
    pPID->output_min = min;
    pPID->output_max = max;
    pPID->limits_enabled = true;
}

/**
 * @brief Reset PID controller internal state
 * @param pPID Pointer to PID structure
 */
void PID_Reset(PID *pPID)
{
    if (pPID == NULL)
        return;
    
    pPID->integral = 0.0f;
    pPID->prev_error = 0.0f;
    pPID->first_run = true;
    pPID->values.control_val = 0.0f;
}

/**
 * @brief Update PID controller calculation
 * @param pPID Pointer to PID structure
 * @return true if calculation was performed, false otherwise
 * @note This function should be called at regular intervals defined by sampling_time_ms
 */
bool PID_Update(PID *pPID)
{
    if (pPID == NULL)
        return false;
    
    uint32_t currentTime = HAL_GetTick();
    
    // Calculate time difference in seconds (use configured sampling time)
    float dt = (float)pPID->sampling_time_ms / 1000.0f;
    
    // Calculate error
    float error = pPID->values.setpoint - pPID->values.current_val;
    
    // Proportional term
    float proportional = pPID->parameters.Kp * error;
    
    // Integral term
    pPID->integral += error * dt;
    float integral = pPID->parameters.Ki * pPID->integral;
    
    // Derivative term
    float derivative = 0.0f;
    if (!pPID->first_run)
    {
        derivative = pPID->parameters.Kd * (error - pPID->prev_error) / dt;
    }
    else
    {
        pPID->first_run = false;
    }
    
    // Calculate PID output
    float output = proportional + integral + derivative;
    
    // Apply output limits if enabled
    if (pPID->limits_enabled)
    {
        if (output > pPID->output_max)
        {
            output = pPID->output_max;
            // Anti-windup: prevent integral windup when output is saturated
            if (pPID->parameters.Ki != 0.0f)
            {
                pPID->integral = (output - proportional - derivative) / pPID->parameters.Ki;
            }
        }
        else if (output < pPID->output_min)
        {
            output = pPID->output_min;
            // Anti-windup: prevent integral windup when output is saturated
            if (pPID->parameters.Ki != 0.0f)
            {
                pPID->integral = (output - proportional - derivative) / pPID->parameters.Ki;
            }
        }
    }
    
    // Store results
    pPID->values.control_val = output;
    pPID->prev_error = error;
    pPID->prevTime = currentTime;
    
    return true;
}

/**
 * @brief Set PID parameters
 * @param pPID Pointer to PID structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void PID_SetParameters(PID *pPID, float kp, float ki, float kd)
{
    if (pPID == NULL)
        return;
    
    pPID->parameters.Kp = kp;
    pPID->parameters.Ki = ki;
    pPID->parameters.Kd = kd;
}

/**
 * @brief Set PID setpoint
 * @param pPID Pointer to PID structure
 * @param setpoint Desired value
 */
void PID_SetSetpoint(PID *pPID, float setpoint)
{
    if (pPID == NULL)
        return;
    
    pPID->values.setpoint = setpoint;
}