#include "pid.h"

// Constructor
PID::PID(float kp, float ki, float kd, float sampleTimeMs)
    : _kp(kp), _ki(ki), _kd(kd), _sampleTimeMs(sampleTimeMs),
      _previousError(0.0f), _integral(0.0f), _lastInput(0.0f), 
      _lastOutput(0.0f), _lastTime(0),
      _outputMin(-255.0f), _outputMax(255.0f),
      _integralMin(-100.0f), _integralMax(100.0f) {
}

// Set PID tunings
void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Set sample time
void PID::setSampleTime(float sampleTimeMs) {
    if (sampleTimeMs > 0) {
        _sampleTimeMs = sampleTimeMs;
    }
}

// Set output limits
void PID::setOutputLimits(float min, float max) {
    if (min < max) {
        _outputMin = min;
        _outputMax = max;
    }
}

// Set integral limits (anti-windup)
void PID::setIntegralLimits(float min, float max) {
    if (min < max) {
        _integralMin = min;
        _integralMax = max;
        
        // Clamp current integral if needed
        if (_integral > _integralMax) _integral = _integralMax;
        else if (_integral < _integralMin) _integral = _integralMin;
    }
}

// Reset PID state
void PID::reset() {
    _previousError = 0.0f;
    _integral = 0.0f;
    _lastInput = 0.0f;
    _lastOutput = 0.0f;
    _lastTime = 0;
}