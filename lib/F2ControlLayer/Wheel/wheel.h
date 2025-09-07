#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>

#include "../F1HardwareLayer/Encoder/encoder.h"
#include "../F1HardwareLayer/Motor/motor.h"
#include "../F2ControlLayer/PID/pid.h"
#include "../F2ControlLayer/Filter/filter.h"

enum WheelDirection {
    FORWARD,
    BACKWARD,
    STOP
};

enum WheelEvent {
    TARGET_REACHED,
    STALLED,
    ERROR
};

class Wheel {

// Public methods
public:
    // Constructor
    Wheel(
        // Motor
        uint8_t pinDir1, uint8_t pinDir2, uint8_t pinPWM, uint16_t pwmFreq,
        // Encoder
        uint8_t pinA, uint8_t pinB, uint16_t reducer_ratio, uint16_t encoder_resolution, EncoderMode mode = X4_MODE,
        // PID
        float kp = 1.0, float ki = 0.0, float kd = 0.0, float sampleTimeMs = 10.0
    );

    // Destructor
    ~Wheel();

    // Initialize wheel components
    bool begin();

    // Stop wheel movement and release resources
    void end();

    // Basic control
    void setSpeed(float speed); // speed in percentage (-100.00 to 100.00)
    void setTargetRPM(float rpm);
    void stop();
    void setRawMotorSpeed(float speed);

    // PID Configuration
    void setPIDTunings(float kp, float ki, float kd);
    void setPIDSampleTime(float sampleTimeMs);
    void setPIDLimits(float min, float max);
    void enablePID(bool enable = true);
    void resetPID();

    // Wheel configuration
    void setWheelDiameter(float diameter);
    void setDirection(bool reversed);

    // Update and Callback
    void update();
    void setCallback(void(*callback)(WheelEvent event, float value));

    // Filter
    void setRPMFilter(Filter* newFilter);
    void enableAdaptiveFilter(bool enable = true);
    void setAdaptiveFilterParams(float lowSpeedThreshold, float highSpeedThreshold);
    
// Public getters
public:
    // Public getters for speed and RPM
    float getCurrentRPM();
    float getCurrentSpeed(); // mm/s
    float getTargetRPM();

    // Public status checks
    bool isAtTarget();
    bool isRotating();
    bool isPIDEnabled();
    long getPulsePosition() const { return _encoder->getPulsePosition(); }
    WheelDirection getDirection();

    // Debugging
    void printStatus(bool active, uint8_t print_interval);
    String getStatusString();

// Private variables
private:
    // Components
    Encoder * _encoder;
    Motor * _motor;
    PID * _pid;
    Filter * _rpmFilter;

    // Hardware config
    uint8_t _pinDir1, _pinDir2, _pinPWM, _pinA, _pinB;
    uint16_t _reducer_ratio, _encoder_resolution, _pwmFreq;
    EncoderMode _mode;
    float _kp = 1.0, _ki = 0.0, _kd = 0.0, _sampleTimeMs = 10.0;

    // Wheel properties
    float _wheelDiameter;       // mm
    bool _reversed;            // Direction flag
    bool _pidEnabled;

    // Control targets
    float _targetRPM;

    // Current state
    float _currentRPM = 0.0f;
    float _currentRPMFiltered = 0.0f;

    WheelDirection _direction;

    // Timing
    unsigned long _lastUpdateTime;
    unsigned long _lastRPMCalcTime;
    unsigned long _rpmCalcInterval;    // ms
    unsigned long _pidUpdateInterval;  // ms

    // RPM calculation
    long _lastPulseCount;
    unsigned long _lastPulseTime;
    float _totalPulsesPerRev;
    float _rpmConversionFactor;
    float _wheelCircumference;
    unsigned long _tempCurrentTime;
    long _tempCurrentPulses; 
    unsigned long _tempDeltaTime;
    long _tempDeltaPulses;

    // Callback
    void (*_callback)(WheelEvent event, float value);

    // Debugging
    uint8_t _printCounter;

// Private methods
private:
    // Calculations
    float pulseToMM(long pulses);    // Keep for getCurrentSpeed()
    long mmToPulse(float mm);        // Keep for wheel diameter calculations

    // Validation
    float constrainSpeed(float speed);

    // Internal control
    void updatePID();
    void updateRPMCalculation();


};

#endif