#ifndef _PID_OUTPUT_H
#define _PID_OUTPUT_H

#include <stdint.h>
#include <stdbool.h>

// Enumy dla motorów (zgodne z pid_parameters.h)
typedef enum {
    MOTOR_ROTATION = 0,
    MOTOR_ELEVATION = 1,
    MOTOR_COUNT = 2
} MotorID;

// Rozszerzona struktura przechowująca wyjścia PID
typedef struct {
    // Speed PID outputs
    float speedOutput;           // Aktualne wyjście speed PID
    float speedRawOutput;        // Speed wyjście przed ograniczeniem
    bool speedIsLimited;         // Czy speed wyjście jest ograniczone
    
    // Position PID outputs (dla trybu kaskadowego)
    float positionOutput;        // Wyjście position PID (speed setpoint)
    float positionError;         // Błąd pozycji
    bool positionIsActive;       // Czy kontroler pozycji jest aktywny
    
    // Final output
    float finalOutput;           // Finalne wyjście na silnik
    
    // Feedback values
    float currentPosition;       // Aktualna pozycja
    float currentSpeed;          // Aktualna prędkość
    
    // Setpoints
    float positionSetpoint;      // Zadana pozycja
    float speedSetpoint;         // Zadana prędkość
    
    // Status
    bool isActive;              // Czy PID jest aktywny
    bool positionControlMode;   // Tryb kontroli pozycji
    uint32_t lastUpdate;        // Timestamp ostatniej aktualizacji
} PIDOutput;

// Globalna struktura dostępna z extern
extern PIDOutput pidOutputs[MOTOR_COUNT];

// Funkcje do zarządzania wyjściami PID
void pidOutputInit(void);

// Speed PID functions
void pidOutputSetSpeed(MotorID motorId, float output, float rawOutput);

// Position PID functions  
void pidOutputSetPosition(MotorID motorId, float output, float error);
void pidOutputSetPositionActive(MotorID motorId, bool active);

// Feedback and setpoints
void pidOutputSetFeedback(MotorID motorId, float position, float speed);
void pidOutputSetSetpoints(MotorID motorId, float positionSP, float speedSP);

// Final output
void pidOutputSetFinal(MotorID motorId, float output);

// Control mode
void pidOutputSetControlMode(MotorID motorId, bool positionControl);

// General control
void pidOutputSetActive(MotorID motorId, bool active);

// Getters
float pidOutputGetSpeed(MotorID motorId);
float pidOutputGetPosition(MotorID motorId);
float pidOutputGetFinal(MotorID motorId);
bool pidOutputIsActive(MotorID motorId);
bool pidOutputIsSpeedLimited(MotorID motorId);
bool pidOutputIsPositionActive(MotorID motorId);
bool pidOutputIsPositionMode(MotorID motorId);

// Debug getters
float pidOutputGetCurrentPosition(MotorID motorId);
float pidOutputGetCurrentSpeed(MotorID motorId);
float pidOutputGetPositionSetpoint(MotorID motorId);
float pidOutputGetSpeedSetpoint(MotorID motorId);
float pidOutputGetPositionError(MotorID motorId);

#endif