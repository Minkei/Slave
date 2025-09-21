#include "wheel.h"
#include <Arduino.h>
#include <math.h>


Wheel::Wheel(uint8_t pinDir1, uint8_t pinDir2, uint8_t pinPWM, uint16_t pwmFreq, 
             uint8_t pinA, uint8_t pinB, uint16_t reducer_ratio, uint16_t encoder_resolution, 
             EncoderMode mode, float kp, float ki, float kd, float sampleTimeMs)
{
    // Initialize hardware configuration
    // Motor
    _pinDir1 = pinDir1;
    _pinDir2 = pinDir2;
    _pinPWM = pinPWM;
    _pwmFreq = pwmFreq;
    
    // Encoder
    _pinA = pinA;
    _pinB = pinB;
    _reducer_ratio = reducer_ratio;
    _encoder_resolution = encoder_resolution;
    _mode = mode;
    
    // PID
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _sampleTimeMs = sampleTimeMs;

    // Initialize components
    _encoder = new Encoder(_pinA, _pinB, _reducer_ratio, _encoder_resolution, _mode);
    _motor = new Motor(_pinDir1, _pinDir2, _pinPWM, _pwmFreq);
    _pid = new PID(_kp, _ki, _kd, _sampleTimeMs);

    // Filter
    _rpmFilter = FilterFactory::createRPMFilter();

    // Wheel properties
    _wheelDiameter = 0; // Default wheel diameter
    _reversed = false;
    _pidEnabled = false;

    // Control targets
    
    _targetRPM = 0.0f;

    // Current state
    _currentRPM = 0.0f;
    _direction = STOP;

    // Timing
    _lastUpdateTime = 0;
    _lastRPMCalcTime = 0;
    _rpmCalcInterval = 10;    // Default 10ms
    _pidUpdateInterval = 10;  // Default 10ms

    // RPM calculation
    _lastPulseCount = 0;
    _lastPulseTime = 0;
    _totalPulsesPerRev = (float)_encoder_resolution * _reducer_ratio * _mode;
    _rpmConversionFactor = 60000.0f / _totalPulsesPerRev;
    _wheelCircumference = PI * _wheelDiameter;

    // Callbacks
    _callback = nullptr;


    // Acceleration limiting initialization
    _maxAcceleration = 60.0f;    
    _currentTargetRPM = 0.0f;
    _finalTargetRPM = 0.0f;
    _lastAccelUpdate = 0;
    _accelerationEnabled = true;
    _maxRPMChange = _maxAcceleration * (sampleTimeMs/1000.0f);
}

Wheel::~Wheel() {
    // Stop the wheel before destruction
    stop();

    if (_encoder) {
        delete _encoder;
        _encoder = nullptr;
    }

    if (_motor) {
        delete _motor;
        _motor = nullptr;
    }

    if (_pid) {
        delete _pid;
        _pid = nullptr;
    }

    if (_rpmFilter) {
        delete _rpmFilter;
        _rpmFilter = nullptr;
    }
}

bool Wheel::begin() {
    bool success = true;
    
    if (_encoder) {
        success &= _encoder->begin();
        if (!success) {
            Serial.println("Failed to initialize encoder.");
            return false;
        }
    }

    if (_motor) {
        _motor->begin();
    }

    if (_pid) {
        _pid->setOutputLimits(-100, 100); // Output range -100% to +100%
        _pid->setSampleTime(_sampleTimeMs);
        _pid->reset();
    }

    _lastUpdateTime = millis();
    _lastRPMCalcTime = millis();
    _lastPulseTime = millis();

    if (_encoder) {
        _lastPulseCount = _encoder->getPulsePosition();
    }
    
    return success;
}

void Wheel::end() {
    stop();

    if (_encoder) {
        _encoder->end();
    }

    if (_pid) {
        _pid->reset();
    }

    _pidEnabled = false;
}

void Wheel::setTargetRPM(float rpm) {
    _finalTargetRPM = rpm;
    
    if (!_accelerationEnabled) {
        // Direct setting if acceleration limiting disabled
        _currentTargetRPM = rpm;
        _targetRPM = rpm;
        _pidEnabled = true;
        
        if (_pid) {
            _pid->reset();
        }
        
        // Set direction
        if (_targetRPM > 0) {
            _direction = FORWARD;
        } else if (_targetRPM < 0) {
            _direction = BACKWARD;
        } else {
            _direction = STOP;
        }
        return;
    }
    
    // With acceleration limiting - will be ramped in update()
    _pidEnabled = true;
    _lastAccelUpdate = millis();
    
    Serial.print("Target RPM set to ");
    Serial.print(_finalTargetRPM);
    Serial.print(" (will ramp from ");
    Serial.print(_currentTargetRPM);
    Serial.println(")");
}

void Wheel::stop()
{
    // Stop motor immediately
    if (_motor) {
        _motor->stop();
    }
    
    // Reset PID state
    if (_pid) {
        _pid->reset();
    }
    
    // Disable PID
    _pidEnabled = false;
    
    // Clear targets
    _targetRPM = 0.0f;
    
    // Update state
    _direction = STOP;
    _currentRPM = 0.0f;
}

void Wheel::setRawMotorSpeed(float speed)
{
    if (_motor) {
        _motor->run(speed);
    }
}

void Wheel::setPIDTunings(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    if (_pid) {
        _pid->setTunings(kp, ki, kd);
    }
}

void Wheel::setPIDSampleTime(float sampleTimeMs)
{
    _sampleTimeMs = sampleTimeMs;
    _pidUpdateInterval = (unsigned long)sampleTimeMs;
    if(_pid) {
        _pid->setSampleTime(sampleTimeMs);
    }   
}

void Wheel::setPIDLimits(float min, float max)
{
    if (_pid) {
        _pid->setOutputLimits(min, max);
    }
}

void Wheel::enablePID(bool enable)
{
    _pidEnabled = enable;

    if(!enable) {
        if(_pid) _pid->reset();
        if(_motor) _motor->stop();
        _direction = STOP;
    }
    
}

void Wheel::resetPID()
{
    if(_pid) {
        _pid->reset();
    }
}

void Wheel::setDirection(bool reversed)
{
    _reversed = reversed;
}


void Wheel::update() {
    unsigned long currentTime = millis();
    
    // === ACCELERATION LIMITING ===
    if (_accelerationEnabled && _pidEnabled) {
        if (currentTime - _lastAccelUpdate >= _sampleTimeMs) { // 20ms update rate
            _lastAccelUpdate = currentTime;
            
            float rpmDifference = _finalTargetRPM - _currentTargetRPM;
            
            if (abs(rpmDifference) > _maxRPMChange) {
                // Need to ramp
                if (rpmDifference > 0) {
                    _currentTargetRPM += _maxRPMChange;
                } else {
                    _currentTargetRPM -= _maxRPMChange;
                }
            } else {
                // Close enough, set directly
                _currentTargetRPM = _finalTargetRPM;
            }
            
            // Update the actual target used by PID
            _targetRPM = _currentTargetRPM;
            
            // Update direction based on current target
            if (_targetRPM > 1.0f) {
                _direction = FORWARD;
            } else if (_targetRPM < -1.0f) {
                _direction = BACKWARD;
            } else {
                _direction = STOP;
            }
        }
    }
    
    // === EXISTING PID UPDATE ===
    if (!_pidEnabled || !_pid) return;

    if (currentTime - _lastUpdateTime < _pidUpdateInterval) return;
    _lastUpdateTime = currentTime;

    float currentRPM = getCurrentRPM();
    float pidOutput = _pid->compute(_targetRPM, currentRPM);

    if (pidOutput) {
        pidOutput = constrain(pidOutput, -100.0f, 100.0f);
        
        if (_motor) {
            _motor->run(pidOutput);
        }
        
        // Direction based on PID output
        if (pidOutput > 1.0f) {
            _direction = FORWARD;
        } else if (pidOutput < -1.0f) {
            _direction = BACKWARD;
        } else {
            _direction = STOP;
        }
    }
}

void Wheel::setRPMFilter(Filter *newFilter)
{
    if(_rpmFilter) {
        delete _rpmFilter;
    }
    _rpmFilter = newFilter;
}


float Wheel::getCurrentRPM() const
{
    if (!_encoder) return 0.0f;
    
    noInterrupts();
    _tempCurrentTime = millis();
    _tempCurrentPulses = _encoder->getPulsePosition();
    interrupts();

    _tempDeltaTime = _tempCurrentTime - _lastPulseTime;

    if (_tempDeltaTime < 10) return _currentRPM; // Avoid too frequent calculations

    _tempDeltaPulses = _tempCurrentPulses - _lastPulseCount;

    _currentRPM = (_tempDeltaPulses * _rpmConversionFactor) / _tempDeltaTime;
    
    if (_reversed) _currentRPM = -_currentRPM;

    _lastPulseTime = _tempCurrentTime;
    _lastPulseCount = _tempCurrentPulses;
    
    

    if (_rpmFilter) {
        _currentRPMFiltered = _rpmFilter->update(_currentRPM);
        return _currentRPMFiltered;
    }

    return _currentRPM;
}

float Wheel::getTargetRPM() const
{
    return _targetRPM;
}

bool Wheel::isAtTarget() const
{
    if (!_pidEnabled) return false;
    float currentRPM = getCurrentRPM();
    float error = abs(_targetRPM - currentRPM);
    static const float RPM_TOLERANCE = 3.0f;
    return error < RPM_TOLERANCE;
}

bool Wheel::isRotating()
{
    float currentRPM = getCurrentRPM();
    static const float MIN_ROTATION_RPM = 2.0f;
    return abs(currentRPM) > MIN_ROTATION_RPM;
}

bool Wheel::isPIDEnabled()
{
    return _pidEnabled;
}

void Wheel::printStatus(bool active, uint8_t print_interval)
{
    if (!active) return;
    _printCounter++;
    if (_printCounter == print_interval) {
        Serial.print("SP:");
        Serial.print(_targetRPM);
        Serial.print(",");
        Serial.print("C:");
        Serial.print(_currentRPMFiltered);
        Serial.println();
        _printCounter = 0;
    }
    
}

void Wheel::updatePID()
{
    // This function is now integrated into update()
    if (!_pidEnabled || !_pid) return;

    float currentRPM = getCurrentRPM();
    float error = _targetRPM - currentRPM;

    static const float RPM_TOLERANCEE = 3.0f;
    bool targetReached = abs(error) < RPM_TOLERANCEE;

    if (targetReached && _callback) {
        _callback(TARGET_REACHED, currentRPM);
    }
}

void Wheel::setMaxAcceleration(float rpmPerSecond)
{
    if(rpmPerSecond > 0) {
        _maxAcceleration = rpmPerSecond;

        //Calculate max change per update cycle (assuming 20ms update interval)
        _maxRPMChange = _maxAcceleration * (_sampleTimeMs / 1000.0f);
        Serial.print("Wheel max acceleration set to ");
        Serial.print(_maxAcceleration);
        Serial.print(" rpm/s, max change per cycle: ");
        Serial.print(_maxRPMChange);
        Serial.println(" rpm");
    }
}

void Wheel::enableAccelerationLimiting(bool enable)
{
    _accelerationEnabled = enable;
    if (!enable) {
        // If disabled, set current target directly to final target
        _currentTargetRPM = _finalTargetRPM;
    }
    Serial.print("Acceleration limiting ");
    Serial.println(enable ? "ENABLED" : "DISABLED");
}
