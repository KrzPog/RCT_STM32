#ifndef _PID_PARAMETERS_H
#define _PID_PARAMETERS_H

// PID Parameters structure for each motor axis
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integralMin;
    float integralMax;
} MotorPIDParams;

// Speed PID parameters for each motor
static const MotorPIDParams MOTOR_SPEED_PID_PARAMS[2] = {
    // MOTOR_ROTATION - Speed control
    {
        .Kp = 0.8f,
        .Ki = 0.4f,  // Lower Ki for rotation (less external forces)
        .Kd = 0.1f,
        .integralMin = -500.0f,
        .integralMax = 500.0f
    },
    // MOTOR_ELEVATION - Speed control
    {
        .Kp = 0.8f,
        .Ki = 0.6f,  // Higher Ki for elevation (gravity compensation)
        .Kd = 0.1f,
        .integralMin = -500.0f,
        .integralMax = 500.0f
    }
};

// Position PID parameters for each motor - now separate and configurable
static const MotorPIDParams MOTOR_POSITION_PID_PARAMS[2] = {
    // MOTOR_ROTATION - Position control
    {
        .Kp = 2.0f,     // Higher Kp for position (stiffer response)
        .Ki = 0.1f,     // Lower Ki for position (avoid integral windup)
        .Kd = 0.5f,     // Higher Kd for position (damping)
        .integralMin = -50.0f,
        .integralMax = 50.0f
    },
    // MOTOR_ELEVATION - Position control
    {
        .Kp = 2.5f,     // Different Kp for elevation (accounting for gravity)
        .Ki = 0.2f,     // Higher Ki for elevation (gravity compensation)
        .Kd = 0.6f,     // Different Kd for elevation (different inertia)
        .integralMin = -75.0f,  // Different integral limits
        .integralMax = 75.0f
    }
};

// Legacy alias for compatibility
#define MOTOR_PID_PARAMS MOTOR_SPEED_PID_PARAMS

#endif