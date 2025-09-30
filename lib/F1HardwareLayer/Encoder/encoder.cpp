#include <Arduino.h>
#include "encoder.h"

// Lookup table cho x4 mode - tối ưu hóa tốc độ
// Index: (lastState << 2) | currentState
// Value: direction (-1, 0, +1)
const int8_t Encoder::_stateTable[16] = {
     0,  // 0000: No change
    +1,  // 0001: A=0,B=0 -> A=0,B=1 (CCW)
    -1,  // 0010: A=0,B=0 -> A=1,B=0 (CW)
     0,  // 0011: Invalid transition
    -1,  // 0100: A=0,B=1 -> A=0,B=0 (CW)  
     0,  // 0101: No change
     0,  // 0110: Invalid transition
    +1,  // 0111: A=0,B=1 -> A=1,B=1 (CCW)
    +1,  // 1000: A=1,B=0 -> A=0,B=0 (CCW)
     0,  // 1001: Invalid transition
     0,  // 1010: No change
    -1,  // 1011: A=1,B=0 -> A=1,B=1 (CW)
     0,  // 1100: Invalid transition  
    -1,  // 1101: A=1,B=1 -> A=0,B=1 (CW)
    +1,  // 1110: A=1,B=1 -> A=1,B=0 (CCW)
     0   // 1111: No change
};

// Static members
Encoder* Encoder::_instances[2] = {nullptr, nullptr};
uint8_t Encoder::_instanceCount = 0;

// Constructor
Encoder::Encoder(uint8_t pinA, uint8_t pinB, uint16_t reducer_ratio, uint16_t encoder_resolution, EncoderMode mode) 
    : _pinA(pinA), _pinB(pinB), _reducer_ratio(reducer_ratio), _encoder_resolution(encoder_resolution), _mode(mode), _pulse_position(0), _lastStateA(0), _lastStateB(0), _lastCombined(0),_callback(nullptr), _lastChangeTime(0), _timeBetweenPulses(0),_lastDirection(0), _lastInterruptTime(0) 
    {
        _totalPulses = _encoder_resolution * _reducer_ratio * _mode;
}

// Destructor
Encoder::~Encoder() {
    end();
}

// Khởi tạo encoder
bool Encoder::begin() {
    if (_instanceCount >= 2) {
        return false; // Quá nhiều instances
    }
    
    // Thiết lập pins
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    
    // Đọc trạng thái ban đầu
    _lastStateA = digitalRead(_pinA);
    _lastStateB = digitalRead(_pinB);
    _lastCombined = (_lastStateA << 1) | _lastStateB;
    
    // Thêm instance vào array
    _instances[_instanceCount] = this;
    uint8_t instanceIndex = _instanceCount++;
    
    // Gắn interrupts dựa trên mode
    switch (_mode) {
        case X1_MODE:
            // Chỉ interrupt trên chân A, cạnh lên
            switch (instanceIndex) {
                case 0: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA0, RISING); break;
                case 1: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA1, RISING); break;
                case 2: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA2, RISING); break;
                case 3: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA3, RISING); break;
            }
            break;
            
        case X2_MODE:
            // Interrupt trên chân A, cả hai cạnh
            switch (instanceIndex) {
                case 0: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA0, CHANGE); break;
                case 1: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA1, CHANGE); break;
                case 2: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA2, CHANGE); break;
                case 3: attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA3, CHANGE); break;
            }
            break;
            
        case X4_MODE:
            // Interrupt trên cả hai chân A và B
            switch (instanceIndex) {
                case 0: 
                    attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA0, CHANGE);
                    attachInterrupt(digitalPinToInterrupt(_pinB), _handleInterruptB0, CHANGE);
                    break;
                case 1:
                    attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA1, CHANGE);
                    attachInterrupt(digitalPinToInterrupt(_pinB), _handleInterruptB1, CHANGE);
                    break;
                case 2:
                    attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA2, CHANGE);
                    attachInterrupt(digitalPinToInterrupt(_pinB), _handleInterruptB2, CHANGE);
                    break;
                case 3:
                    attachInterrupt(digitalPinToInterrupt(_pinA), _handleInterruptA3, CHANGE);
                    attachInterrupt(digitalPinToInterrupt(_pinB), _handleInterruptB3, CHANGE);
                    break;
            }
            break;
    }
    
    return true;
}

// Dừng encoder
void Encoder::end() {
    uint8_t index = _getInstanceIndex();
    if (index < 4) {
        detachInterrupt(digitalPinToInterrupt(_pinA));
        if (_mode == X4_MODE) {
            detachInterrupt(digitalPinToInterrupt(_pinB));
        }
        
        // Xoá instance khỏi array
        for (uint8_t i = index; i < _instanceCount - 1; i++) {
            _instances[i] = _instances[i + 1];
        }
        _instances[--_instanceCount] = nullptr;
    }
}

// Đọc vị trí hiện tại
long Encoder::getPulsePosition() {
    noInterrupts();
    long pos = _pulse_position;
    interrupts();
    return pos;
}

float Encoder::getAnglePosition(AngleUnit unit) {
    noInterrupts();
    long pos = _pulse_position;
    interrupts();

    if (unit == RADIAN) {
        return ((float)pos / _totalPulses)*TWO_PI;
    }
    return ((float)pos / _totalPulses) * 360.0;
}

void Encoder::setAnglePosition(float angle, AngleUnit unit)
{
    noInterrupts();
    long pos = _pulse_position;
    interrupts();

    
    if (unit == RADIAN) {
        pos = (long)((angle / TWO_PI) * _totalPulses);
    } else {
        pos = (long)((angle / 360.0) * _totalPulses);
    }

    setPulsePosition(pos);
}

// Set vị trí encoder  
void Encoder::setPulsePosition(long pulse_position) {
    noInterrupts();
    _pulse_position = pulse_position;
    interrupts();
}

// Reset vị trí về 0
void Encoder::resetPulsePosition() {
    setPulsePosition(0);
}





// Kiểm tra encoder có đang quay không
bool Encoder::isRotating() {
    return (micros() - _lastChangeTime) < 50000; // 50ms timeout
}

// Set callback function
void Encoder::setCallback(void (*callback)(long position, int8_t direction)) {
    _callback = callback;
}

// Đọc mode hiện tại
EncoderMode Encoder::getMode() {
    return _mode;
}

// Tìm instance index
uint8_t Encoder::_getInstanceIndex() {
    for (uint8_t i = 0; i < _instanceCount; i++) {
        if (_instances[i] == this) {
            return i;
        }
    }
    return 255; // Not found
}

// Instance interrupt handler
void Encoder::_handleInterrupt() {
    unsigned long currentTime = micros();
    
    // Debouncing
    if (currentTime - _lastInterruptTime < _debounceDelay) {
        return;
    }
    _lastInterruptTime = currentTime;
    
    uint8_t stateA = digitalRead(_pinA);
    uint8_t stateB = digitalRead(_pinB);
    int8_t direction = 0;
    
    switch (_mode) {
        case X1_MODE:
            // Chỉ đọc cạnh lên của chân A
            direction = (stateB == LOW) ? 1 : -1;
            _pulse_position += direction;
            break;
            
        case X2_MODE:
            // Đọc cả hai cạnh của chân A
            if (stateA != _lastStateA) {
                direction = (stateA == stateB) ? 1 : -1;
                _pulse_position += direction;
            }
            break;
            
        case X4_MODE:
            // Sử dụng lookup table cho tốc độ tối ưu
            uint8_t combined = (stateA << 1) | stateB;
            uint8_t index = (_lastCombined << 2) | combined;
            direction = _stateTable[index];
            _pulse_position += direction;
            _lastCombined = combined;
            break;
    }
    
    // Cập nhật timing và direction
    if (abs(direction) > 0) {
        _timeBetweenPulses = currentTime - _lastChangeTime;
        _lastChangeTime = currentTime;
        _lastDirection = direction;
        
        // Gọi callback nếu có
        if (_callback) {
            _callback(_pulse_position, direction);
        }
    }
    
    _lastStateA = stateA;
    _lastStateB = stateB;
}

// Static interrupt handlers cho instance 0
void Encoder::_handleInterruptA0() { if (_instances[0]) _instances[0]->_handleInterrupt(); }
void Encoder::_handleInterruptB0() { if (_instances[0]) _instances[0]->_handleInterrupt(); }

// Static interrupt handlers cho instance 1
void Encoder::_handleInterruptA1() { if (_instances[1]) _instances[1]->_handleInterrupt(); }
void Encoder::_handleInterruptB1() { if (_instances[1]) _instances[1]->_handleInterrupt(); }

// Static interrupt handlers cho instance 2
void Encoder::_handleInterruptA2() { if (_instances[2]) _instances[2]->_handleInterrupt(); }
void Encoder::_handleInterruptB2() { if (_instances[2]) _instances[2]->_handleInterrupt(); }

// Static interrupt handlers cho instance 3  
void Encoder::_handleInterruptA3() { if (_instances[3]) _instances[3]->_handleInterrupt(); }
void Encoder::_handleInterruptB3() { if (_instances[3]) _instances[3]->_handleInterrupt(); }