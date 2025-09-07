#include "motor.h"

// Constructor
Motor::Motor(uint8_t pinDir1, uint8_t pinDir2, uint8_t pinPWM, uint16_t pwmFreq) 
    : _pinDir1(pinDir1), _pinDir2(pinDir2), _pinPWM(pinPWM), _pwmFreq(pwmFreq) {
}

// Khởi tạo motor
void Motor::begin() {
    pinMode(_pinDir1, OUTPUT);
    pinMode(_pinDir2, OUTPUT);
    pinMode(_pinPWM, OUTPUT);
    
    // Set PWM frequency
    analogWriteFreq(_pwmFreq);
    
    stop();
}

// Điều khiển motor (-100 đến +100%)
void Motor::run(float speed) {
    // Constrain speed
    speed = constrain(speed, -100.0f, 100.0f);

    if (abs(speed) < 0.01f) {  // Threshold instead of exact comparison
        stop();
        return;
    }
    
    // Calculate PWM with float precision
    uint8_t pwmValue = (uint8_t)((abs(speed) * 255.0f) / 100.0f);
    
    // Set direction
    if (speed > 0) {
        digitalWrite(_pinDir1, HIGH);
        digitalWrite(_pinDir2, LOW);
    } else {
        digitalWrite(_pinDir1, LOW);
        digitalWrite(_pinDir2, HIGH);
    }
    
    // Set PWM
    analogWrite(_pinPWM, pwmValue);
}

// Dừng motor
void Motor::stop() {
    digitalWrite(_pinDir1, LOW);
    digitalWrite(_pinDir2, LOW);
    analogWrite(_pinPWM, 0);
}