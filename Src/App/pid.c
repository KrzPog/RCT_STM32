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
    pPID->values.setpoint = 0;
    pPID->values.current_val = 0;
    pPID->values.control_val = 0;
    
    // Clear out the internal calculation variables
    pPID->integral_scaled = 0;
    pPID->prev_error = 0;
    pPID->first_run = true;
    pPID->prevTime = HAL_GetTick();
    pPID->sampling_time_ms = sampling_time_ms;
    
    // Start with output limits disabled
    pPID->output_min = 0;
    pPID->output_max = 0;
    pPID->limits_enabled = false;
    
    // Start with speed limits disabled
    pPID->max_speed_up = 0;
    pPID->max_speed_down = 0;
    pPID->speed_limits_enabled = false;
    pPID->prev_output = 0;
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
void PID_SetLimits(PID *pPID, int16_t min, int16_t max)
{
    if (pPID == NULL)
        return;
    
    pPID->output_min = min;
    pPID->output_max = max;
    pPID->limits_enabled = true;
}

/**
 * Sets up speed limits to control how fast the output can change
 * 
 * This prevents sudden jumps in output that could cause system instability
 * or mechanical stress. Limits are applied per update cycle.
 * 
 * @param pPID           Which PID controller to limit
 * @param max_speed_up   Maximum increase in output per update cycle
 * @param max_speed_down Maximum decrease in output per update cycle (positive value)
 */
void PID_SetSpeedLimits(PID *pPID, int16_t max_speed_up, int16_t max_speed_down)
{
    if (pPID == NULL)
        return;
    
    pPID->max_speed_up = max_speed_up;
    pPID->max_speed_down = max_speed_down;
    pPID->speed_limits_enabled = true;
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
    pPID->integral_scaled = 0;
    pPID->prev_error = 0;
    pPID->first_run = true;
    pPID->values.control_val = 0;
    pPID->prev_output = 0;
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
    
    // POPRAWKA: Normalizacja błędu dla enkodera
    int16_t raw_error = pPID->values.setpoint - pPID->values.current_val;
    int16_t error = normalize_angle_error(raw_error);
    
    // Proportional
    int32_t proportional = (int32_t)(pPID->parameters.Kp * error);
    
    // Integral
    pPID->integral_scaled += (int32_t)(error * dt * PID_SCALE_FACTOR);
    int32_t integral = (int32_t)(pPID->parameters.Ki * pPID->integral_scaled / PID_SCALE_FACTOR);
    
    // Derivative - POPRAWKA: normalizacja różnicy błędów
    int32_t derivative = 0;
    if (!pPID->first_run)
    {
        int16_t error_diff = normalize_angle_error(error - pPID->prev_error);
        derivative = (int32_t)(pPID->parameters.Kd * error_diff / dt);
    }
    else
    {
        pPID->first_run = false;
    }
    
    // Combine all three terms
    int32_t output_32 = proportional + integral + derivative;
    
    // Clamp to int16_t range
    if (output_32 > INT16_MAX)
        output_32 = INT16_MAX;
    else if (output_32 < INT16_MIN)
        output_32 = INT16_MIN;
    
    int16_t output = (int16_t)output_32;
    
    // NAJPIERW zastosuj speed limits (jeśli włączone)
    if (pPID->speed_limits_enabled && !pPID->first_run)
    {
        int16_t output_change = output - pPID->prev_output;
        
        if (output_change > pPID->max_speed_up)
        {
            output = pPID->prev_output + pPID->max_speed_up;
        }
        else if (output_change < -(int16_t)pPID->max_speed_down)
        {
            output = pPID->prev_output - pPID->max_speed_down;
        }
    }
    
    // POTEM zastosuj output limits i anti-windup
    if (pPID->limits_enabled)
    {
        if (output > pPID->output_max)
        {
            output = pPID->output_max;
            // Anti-windup
            if (pPID->parameters.Ki != 0.0f)
            {
                int32_t max_integral = (int32_t)((output - proportional - derivative) / pPID->parameters.Ki * PID_SCALE_FACTOR);
                if (pPID->integral_scaled > max_integral)
                    pPID->integral_scaled = max_integral;
            }
        }
        else if (output < pPID->output_min)
        {
            output = pPID->output_min;
            // Anti-windup
            if (pPID->parameters.Ki != 0.0f)
            {
                int32_t min_integral = (int32_t)((output - proportional - derivative) / pPID->parameters.Ki * PID_SCALE_FACTOR);
                if (pPID->integral_scaled < min_integral)
                    pPID->integral_scaled = min_integral;
            }
        }
    }
    
    // Save the results - POPRAWKA: zapisz znormalizowany błąd
    pPID->values.control_val = output;
    pPID->prev_error = error;  // Zapisz znormalizowany błąd
    pPID->prev_output = output;
    pPID->prevTime = currentTime;
    
    return true;
}

/**
 * Normalizuje błąd kąta dla enkodera 0-360°
 * Zapewnia, że błąd jest zawsze w zakresie -180° do +180°
 * 
 * @param error  Surowy błąd (setpoint - current)
 * @return       Znormalizowany błąd (-180° do +180°)
 */
static int16_t normalize_angle_error(int16_t error)
{
    // Dla enkodera 0-360°: normalizujemy do ±180°
    while (error > 180)
        error -= 360;
    while (error < -180)
        error += 360;
    
    return error;
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
void PID_SetSetpoint(PID *pPID, int16_t setpoint)
{
    if (pPID == NULL)
        return;
    
    pPID->values.setpoint = setpoint;
}