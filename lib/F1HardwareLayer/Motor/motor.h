#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
    uint8_t _pinDir1;
    uint8_t _pinDir2;
    uint8_t _pinPWM;
    uint16_t _pwmFreq;

public:
    // Constructor
    Motor(uint8_t pinDir1, uint8_t pinDir2, uint8_t pinPWM, uint16_t pwmFreq);
    
    // Khởi tạo motor
    void begin();
    
    // Điều khiển motor (-100 đến +100%)
    void run(float speed);
    
    // Dừng motor
    void stop();

    void quickStop();
};

#endif