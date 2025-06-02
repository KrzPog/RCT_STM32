#ifndef _PID_H
#define _PID_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#define PID_SPEED_SAMPLING_TIME_MS    25   //!< PID speed update period in ms
#define PID_POSITION_SAMPLING_TIME_MS 10   //!< PID position update period in ms

typedef struct
{
    float Kp;  //!< Proportional gain
    float Ki;  //!< Integral gain
    float Kd;  //!< Derivative gain
} PID_Parameters;

typedef struct
{
    float setpoint;     //!< Desired value (reference)
    float current_val;  //!< Current process value (feedback)
    float control_val;  //!< PID output (control signal)
    float actuator_val; //!< Value to be sent to actuator (accessible from outside)
} PID_Values;

typedef struct
{
    PID_Parameters parameters; //!< PID gains
    PID_Values values;         //!< PID values
    
    uint32_t prevTime;         //!< Last update time (ms) - for reference only
    uint16_t sampling_time_ms; //!< Sampling time in milliseconds
    
    // Internal PID variables
    float integral;            //!< Integral sum
    float prev_error;          //!< Previous error for derivative calculation
    bool first_run;           //!< Flag to indicate first execution
    
    // Output limits
    float output_min;         //!< Minimum output limit
    float output_max;         //!< Maximum output limit
    bool limits_enabled;      //!< Enable/disable output limiting
    
} PID;

// External PID controller instances
extern PID PID_speed_elev;
extern PID PID_position_elev;
extern PID PID_speed_rot;
extern PID PID_position_rot;

// Function prototypes
void PID_Init(PID *pPID, float kp, float ki, float kd, uint16_t sampling_time_ms);
void PID_SetLimits(PID *pPID, float min, float max);
void PID_Reset(PID *pPID);
bool PID_Update(PID *pPID);
void PID_SetParameters(PID *pPID, float kp, float ki, float kd);
void PID_SetSetpoint(PID *pPID, float setpoint);
void PID_SetActuatorValue(PID *pPID, float actuator_val);

#endif