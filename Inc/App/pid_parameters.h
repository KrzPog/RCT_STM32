#ifndef _PID_PARAMETERS_H
#define _PID_PARAMETERS_H

// PID Parameters for each motor axis
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integralMin;
    float integralMax;
} MotorPIDParams;

// Motor-specific PID parameters
static const MotorPIDParams MOTOR_PID_PARAMS[2] = {
    // MOTOR_ROTATION
    {
        .Kp = 0.8f,
        .Ki = 0.4f,  // Lower Ki for rotation (less external forces)
        .Kd = 0.1f,
        .integralMin = -500.0f,
        .integralMax = 500.0f
    },
    // MOTOR_ELEVATION  
    {
        .Kp = 0.8f,
        .Ki = 0.6f,  // Higher Ki for elevation (gravity compensation)
        .Kd = 0.1f,
        .integralMin = -500.0f,
        .integralMax = 500.0f
    }
};

#endif