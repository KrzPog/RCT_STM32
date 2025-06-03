#include "App/pid.h"

// Create the actual PID controller instances
PID PID_speed_elev;
PID PID_position_elev;
PID PID_speed_rot;
PID PID_position_rot;

/**
 * Sets up a PID controller with initial settings
 * 
 * Call this once for each PID controller before using it.
 * Sets all internal values to zero and configures the basic parameters.
 * 
 * @param pPID           Which PID controller to set up
 * @param kp             Proportional gain (start with 1.0 and adjust)
 * @param ki             Integral gain (start with 0.0 and increase slowly)
 * @param kd             Derivative gain (start with 0.0, usually small values)
 * @param sampling_time_ms How often you'll call PID_Update() in milliseconds
 */
void PID_Init(PID *pPID, float kp, float ki, float kd, uint16_t sampling_time_ms)
{
    if (pPID == NULL)
        return;
    
    // Store the tuning parameters
    pPID->parameters.Kp = kp;
    pPID->parameters.Ki = ki;
    pPID->parameters.Kd = kd;
    
    // Start with everything at zero
    pPID->values.setpoint = 0.0f;
    pPID->values.current_val = 0.0f;
    pPID->values.control_val = 0.0f;
    
    // Clear out the internal calculation variables
    pPID->integral = 0.0f;
    pPID->prev_error = 0.0f;
    pPID->first_run = true;
    pPID->prevTime = HAL_GetTick();
    pPID->sampling_time_ms = sampling_time_ms;
    
    // Start with output limits disabled
    pPID->output_min = 0.0f;
    pPID->output_max = 0.0f;
    pPID->limits_enabled = false;
}

/**
 * Sets up output limits to keep the controller from going too crazy
 * 
 * This is important for safety - prevents the controller from commanding
 * values that could damage your system or cause dangerous behavior.
 * 
 * @param pPID  Which PID controller to limit
 * @param min   Smallest value the controller can output
 * @param max   Largest value the controller can output
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
 * Clears the controller's memory and starts fresh
 * 
 * Use this when you want to "forget" what happened before - like when
 * switching to a completely different target or after a system disturbance.
 * The controller parameters (Kp, Ki, Kd) stay the same.
 * 
 * @param pPID  Which PID controller to reset
 */
void PID_Reset(PID *pPID)
{
    if (pPID == NULL)
        return;
    
    // Clear the memory but keep the tuning parameters
    pPID->integral = 0.0f;
    pPID->prev_error = 0.0f;
    pPID->first_run = true;
    pPID->values.control_val = 0.0f;
}

/**
 * This is where the magic happens - calculates what the controller should do
 * 
 * Call this function regularly (every sampling_time_ms milliseconds).
 * Make sure to update current_val with your sensor reading before calling this.
 * After calling, use control_val to drive your actuator (motor, valve, etc).
 * 
 * @param pPID  Which PID controller to update
 * @return      true if calculation succeeded, false if something went wrong
 */
bool PID_Update(PID *pPID)
{
    if (pPID == NULL)
        return false;
    
    uint32_t currentTime = HAL_GetTick();
    
    // Use the configured sampling time for consistent calculations
    float dt = (float)pPID->sampling_time_ms / 1000.0f;
    float error = pPID->values.setpoint - pPID->values.current_val;
    
    // Proportional
    float proportional = pPID->parameters.Kp * error;
    
    // Integral
    pPID->integral += error * dt;
    float integral = pPID->parameters.Ki * pPID->integral;
    
    // Derivative
    float derivative = 0.0f;
    if (!pPID->first_run)
    {
        // Calculate rate of change of error
        derivative = pPID->parameters.Kd * (error - pPID->prev_error) / dt;
    }
    else
    {
        // Skip derivative calculation on first run
        pPID->first_run = false;
    }
    
    // Combine all three terms to get the final output
    float output = proportional + integral + derivative;
    
    // Apply safety limits
    if (pPID->limits_enabled)
    {
        if (output > pPID->output_max)
        {
            output = pPID->output_max;
            // Anti-windup
            if (pPID->parameters.Ki != 0.0f)
            {
                pPID->integral = (output - proportional - derivative) / pPID->parameters.Ki;
            }
        }
        else if (output < pPID->output_min)
        {
            output = pPID->output_min;
            // Anti-windup
            if (pPID->parameters.Ki != 0.0f)
            {
                pPID->integral = (output - proportional - derivative) / pPID->parameters.Ki;
            }
        }
    }
    
    // Save the results
    pPID->values.control_val = output;
    pPID->prev_error = error;
    pPID->prevTime = currentTime;
    
    return true;
}

/**
 * Changes the PID tuning parameters while running
 * 
 * Use this to adjust controller behavior without stopping.
 * Be careful - bad parameters can make your system unstable!
 * 
 * @param pPID  Which PID controller to retune
 * @param kp    New proportional gain
 * @param ki    New integral gain  
 * @param kd    New derivative gain
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
 * Tells the controller where you want to go
 * 
 * This sets the target value (reference point) for the controller.
 * The controller will try to make current_val equal to this setpoint.
 * 
 * @param pPID      Which PID controller to command
 * @param setpoint  The target value you want to reach
 */
void PID_SetSetpoint(PID *pPID, float setpoint)
{
    if (pPID == NULL)
        return;
    
    pPID->values.setpoint = setpoint;
}