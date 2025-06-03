#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>


#define PID_SPEED_SAMPLING_TIME_MS    25   // 25ms 
#define PID_POSITION_SAMPLING_TIME_MS 55   // 55ms 

// PID controller gains 
typedef struct
{
    float Kp;  // Proportional gain 
    float Ki;  // Integral gain 
    float Kd;  // Derivative gain 
} PID_Parameters;

// The main values that flow through the PID controller
typedef struct
{
    float setpoint;     // target value
    float current_val;  // measured feedback
    float control_val;  // output signal
} PID_Values;

// Complete PID controller structure
typedef struct
{
    PID_Parameters parameters; // Kp, Ki, Kd
    PID_Values values;         // Input and output values
    
    uint32_t prevTime;         
    uint16_t sampling_time_ms; // sampling time in milliseconds
    
    // Internal calculation variables 
    float integral;            // Running sum of all past errors
    float prev_error;          // Previous error value 
    bool first_run;           
    
    // Safety limits
    float output_min;         
    float output_max;        
    bool limits_enabled;      
    
} PID;

// Pre-made PID controllers for common use cases
extern PID PID_speed_elev;    
extern PID PID_position_elev; 
extern PID PID_speed_rot;     
extern PID PID_position_rot;  

// Functions 
void PID_Init(PID *pPID, float kp, float ki, float kd, uint16_t sampling_time_ms);
void PID_SetLimits(PID *pPID, float min, float max);
void PID_Reset(PID *pPID);
bool PID_Update(PID *pPID);
void PID_SetParameters(PID *pPID, float kp, float ki, float kd);
void PID_SetSetpoint(PID *pPID, float setpoint);

#endif