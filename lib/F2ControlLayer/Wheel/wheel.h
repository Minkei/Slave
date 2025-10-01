#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>

#include "../F1HardwareLayer/Encoder/encoder.h"
#include "../F1HardwareLayer/Motor/motor.h"
#include "../F2ControlLayer/PID/pid.h"
#include "../F2ControlLayer/Filter/filter.h"

enum WheelDirection
{
    FORWARD,
    BACKWARD,
    STOP
};

enum WheelEvent
{
    TARGET_REACHED,
    STALLED,
    ERROR
};

class Wheel
{

    // Public methods
public:
    // Constructor
    Wheel(
        // Motor
        uint8_t pinDir1, uint8_t pinDir2, uint8_t pinPWM, uint16_t pwmFreq,
        // Encoder
        uint8_t pinA, uint8_t pinB, uint16_t reducer_ratio, uint16_t encoder_resolution, EncoderMode mode = X4_MODE,
        // PID
        float kp = 1.0, float ki = 0.0, float kd = 0.0, float sampleTimeMs = 10.0);

    // Destructor
    ~Wheel();

    // Initialize wheel components
    bool begin();

    // Stop wheel movement and release resources
    void end();

    // Basic control
    void setTargetRPM(float rpm);
    void stop();
    void quickStop();
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
    void setCallback(void (*callback)(WheelEvent event, float value));

    // Filter
    void setRPMFilter(Filter *newFilter);
    void enableAdaptiveFilter(bool enable = true);
    void setAdaptiveFilterParams(float lowSpeedThreshold, float highSpeedThreshold);

    // Helper
    void updateGainScheduling();
public:
    // Public getters for speed and RPM
    float getCurrentRPM() const;
    float getCurrentSpeed(); // mm/s
    float getTargetRPM() const;

    // Public status checks
    bool isAtTarget() const;
    bool isRotating();
    bool isPIDEnabled();
    long getPulsePosition() const { return _encoder->getPulsePosition(); }
    WheelDirection getDirection() const;

    // Debugging
    void printStatus(bool active, uint8_t print_interval) const;
    String getStatusString();

    // Private variables
private:
    // Components
    Encoder *_encoder;
    Motor *_motor;
    PID *_pid;
    Filter *_rpmFilter;

    // Hardware config
    uint8_t _pinDir1, _pinDir2, _pinPWM, _pinA, _pinB;
    uint16_t _reducer_ratio, _encoder_resolution, _pwmFreq;
    EncoderMode _mode;
    float _kp = 1.0, _ki = 0.0, _kd = 0.0, _sampleTimeMs = 10.0;

    // Wheel properties
    float _wheelDiameter; // mm
    bool _reversed;       // Direction flag
    bool _pidEnabled;

    // Control targets
    float _targetRPM;

    

    WheelDirection _direction;

    // Timing
    unsigned long _lastUpdateTime;
    unsigned long _lastRPMCalcTime;
    unsigned long _rpmCalcInterval;   // ms
    unsigned long _pidUpdateInterval; // ms

    float _totalPulsesPerRev;
    float _rpmConversionFactor;
    float _wheelCircumference;

    struct RPMCalcCache
    {
        unsigned long currentTime;
        long currentPulses;
        unsigned long deltaTime;
        long deltaPulses;
        long lastPulseCount;
        unsigned long lastPulseTime;
        float currentRPM = 0.0f;
        float currentRPMFiltered = 0.0f;
    } mutable _rpmCalcCache;

    // Callback
    void (*_callback)(WheelEvent event, float value);

    // Debugging
    mutable uint8_t _printCounter;

    // Private methods
private:
    // Calculations
    float pulseToMM(long pulses); // Keep for getCurrentSpeed()
    long mmToPulse(float mm);     // Keep for wheel diameter calculations

    // Validation
    float constrainSpeed(float speed);

    // Internal control
    void updatePID();
    void updateRPMCalculation();

    // Add acc for wheel
private:
    // Acceleration limiting
    float _maxAcceleration;  // RPM/s
    float _currentTargetRPM; // Current target being ramped to
    float _finalTargetRPM;   // Final target set by user
    unsigned long _lastAccelUpdate;
    bool _accelerationEnabled;

    // Power management
    float _maxRPMChange; // Max RPM change per update cycle

public:
    // Acceleration control
    void setMaxAcceleration(float rpmPerSecond);
    void enableAccelerationLimiting(bool enable = true);
    float getMaxAcceleration() const { return _maxAcceleration; }
    bool isAccelerationEnabled() const { return _accelerationEnabled; }

    // Status
    float getFinalTargetRPM() const { return _finalTargetRPM; }
    float getCurrentTargetRPM() const { return _currentTargetRPM; }
    bool isAccelerating() const { return abs(_finalTargetRPM - _currentTargetRPM) > 0.1f; }
};
#endif