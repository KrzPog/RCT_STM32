#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include "main.h"

#define PID_SPEED_SAMPLING_TIME_MS    25   // 25ms 
#define PID_POSITION_SAMPLING_TIME_MS 55   // 55ms 

// Skalowanie dla obliczeń wewnętrznych (Q16.16 fixed point)
#define PID_SCALE_FACTOR 65536  // 2^16 dla precyzji

// PID controller gains - nadal float dla łatwiejszego tuningu
typedef struct
{
    float Kp;  // Proportional gain 
    float Ki;  // Integral gain 
    float Kd;  // Derivative gain 
} PID_Parameters;

// The main values that flow through the PID controller - teraz int16_t
typedef struct
{
    int16_t setpoint;     // target value
    int16_t current_val;  // measured feedback
    int16_t control_val;  // output signal
} PID_Values;

// Complete PID controller structure
typedef struct
{
    PID_Parameters parameters; // Kp, Ki, Kd (float dla łatwiejszego tuningu)
    PID_Values values;         // Input and output values (int16_t)
    
    uint32_t prevTime;         
    uint16_t sampling_time_ms; // sampling time in milliseconds
    
    // Internal calculation variables - używamy int32_t dla większej precyzji
    int32_t integral_scaled;   // Running sum of all past errors (scaled)
    int16_t prev_error;        // Previous error value 
    bool first_run;           
    
    // Safety limits
    int16_t output_min;         
    int16_t output_max;        
    bool limits_enabled;
    
    // Rate limiting (speed limits)
    int16_t max_speed_up;       // Maximum increase per update
    int16_t max_speed_down;     // Maximum decrease per update  
    bool speed_limits_enabled;
    int16_t prev_output;        // Previous output for rate limiting      
    
} PID;

// Pre-made PID controllers for common use cases
extern PID PID_speed_elev;    
extern PID PID_position_elev; 
extern PID PID_speed_rot;     
extern PID PID_position_rot;  

// Functions 
void PID_Init(PID *pPID, float kp, float ki, float kd, uint16_t sampling_time_ms);
void PID_SetLimits(PID *pPID, int16_t min, int16_t max);
void PID_SetSpeedLimits(PID *pPID, int16_t max_speed_up, int16_t max_speed_down);
void PID_Reset(PID *pPID);
bool PID_Update(PID *pPID);
void PID_SetParameters(PID *pPID, float kp, float ki, float kd);
void PID_SetSetpoint(PID *pPID, int16_t setpoint);
static int16_t normalize_angle_error(int16_t error);

#endif