#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
private:
    // PID gains
    float _kp, _ki, _kd;
    
    // Timing
    float _sampleTimeMs;
    uint32_t _lastTime;
    
    // PID state variables
    float _previousError;
    float _integral;
    float _lastInput;        // Move static to instance variable
    float _lastOutput;
    
    // Limits
    float _outputMin, _outputMax;
    float _integralMin, _integralMax;

public:
    PID(float kp, float ki, float kd, float sampleTimeMs = 10.0f);
    
    inline float compute(float setpoint, float input) {
        uint32_t currentTime = millis();
        
        if (currentTime - _lastTime < _sampleTimeMs) {
            return _lastOutput;
        }
        
        float dt = (currentTime - _lastTime) / 1000.0f;
        float error = setpoint - input;
        
        // Proportional
        float pTerm = error * _kp;
        
        // Integral with windup prevention
        _integral += error * dt;
        if (_integral > _integralMax) _integral = _integralMax;
        else if (_integral < _integralMin) _integral = _integralMin;
        float iTerm = _integral * _ki;
        
        // Derivative on measurement
        float dInput = (input - _lastInput) / dt;
        float dTerm = -_kd * dInput;
        
        // Calculate output
        float output = pTerm + iTerm + dTerm;
        
        // Output limiting with integral windup prevention
        if (output > _outputMax) {
            output = _outputMax;
            // Back-calculate integral only if Ki != 0
            if (_ki != 0) {
                _integral = (_outputMax - pTerm - dTerm) / _ki;
            }
        } else if (output < _outputMin) {
            output = _outputMin;
            if (_ki != 0) {
                _integral = (_outputMin - pTerm - dTerm) / _ki;
            }
        }
        
        // Update state
        _lastInput = input;
        _lastTime = currentTime;
        _lastOutput = output;
        
        return output;
    }
    
    void setTunings(float kp, float ki, float kd);
    void setSampleTime(float sampleTimeMs);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void reset();
    
    // Getters
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }
    float getIntegral() const { return _integral; }
};

#endif