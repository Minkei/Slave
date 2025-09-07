#ifndef ENCODER_H
#define ENCODER_H
#include<Arduino.h>

enum EncoderMode {
    X1_MODE = 1,
    X2_MODE = 2,
    X4_MODE = 4
};


enum AngleUnit {
    RADIAN,
    DEGREE
};

class Encoder {
private:
    uint8_t _pinA;
    uint8_t _pinB;
    uint16_t _reducer_ratio;
    uint16_t _encoder_resolution;
    uint32_t _totalPulses;
    EncoderMode _mode;

    volatile long _pulse_position;
    volatile uint8_t _lastStateA;
    volatile uint8_t _lastStateB;
    volatile uint8_t _lastCombined;

    static const int8_t _stateTable[16];
    static Encoder* _instances[2];
    static uint8_t _instanceCount;

    static void _handleInterruptA0();
    static void _handleInterruptA1(); 
    static void _handleInterruptA2();
    static void _handleInterruptA3();
    static void _handleInterruptB0();
    static void _handleInterruptB1();
    static void _handleInterruptB2(); 
    static void _handleInterruptB3();
    
    void _handleInterrupt();
    
    uint8_t _getInstanceIndex();
public:
    // Constructor
    Encoder(uint8_t pinA, uint8_t pinB, uint16_t reducer_ratio, uint16_t encoder_resolution, EncoderMode mode = X4_MODE);
    
    // Destructor
    ~Encoder();
    
    // Khởi tạo encoder
    bool begin();
    
    // Dừng encoder và detach interrupts
    void end();
    
    // Đọc vị trí hiện tại
    long getPulsePosition();
    
    // Set vị trí encoder
    void setPulsePosition(long pulse_position);
    
    // Reset vị trí về 0
    void resetPulsePosition();

    // Đọc vị trí góc hiện tại
    float getAnglePosition(AngleUnit unit = DEGREE);

    // Set vị trí góc hiện tại
    void setAnglePosition(float angle, AngleUnit unit = DEGREE);

    // Kiểm tra xem encoder có đang quay không
    bool isRotating();
    
    // Set callback function khi có thay đổi vị trí
    void setCallback(void (*callback)(long pulse_position, int8_t direction));
    
    // Đọc mode hiện tại
    EncoderMode getMode();
    
private:
    // Callback function pointer
    void (*_callback)(long pulse_position, int8_t direction);
    
    // Biến cho tính tốc độ
    volatile unsigned long _lastChangeTime;
    volatile unsigned long _timeBetweenPulses;
    volatile int8_t _lastDirection;
    
    // Debouncing
    static const uint16_t _debounceDelay = 1; // microseconds
    volatile unsigned long _lastInterruptTime;
};

#endif