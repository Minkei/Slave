// sequenceExecutor.h
#ifndef SEQUENCE_EXECUTOR_H
#define SEQUENCE_EXECUTOR_H

#include <Arduino.h>
#include "../Odometry/odometry.h"

struct PrimitiveRef {
    float x;
    float y;
    float theta;
};

class SequenceExecutor {
private:
    String _commandString;
    int _cmdIndex;
    float _cmdDuration;
    unsigned long _startTime;
    bool _active;
    
    // Cumulative pose tracking
    float _cumulativeX;
    float _cumulativeY;
    float _cumulativeTheta;
    
    // Primitive generators (return local coordinates)
    PrimitiveRef primitiveForward(float t);
    PrimitiveRef primitiveLeft(float t);
    PrimitiveRef primitiveRight(float t);
    
    // Update cumulative position after completing a primitive
    void updateCumulativePosition(char cmd);
    
    // Transform local to global coordinates
    PrimitiveRef transformToGlobal(const PrimitiveRef& local);
    
public:
    SequenceExecutor();
    
    // Setup and control
    void setCommandString(String commands, float duration = 3.0f);
    void start();
    void stop();
    void reset();
    
    // Main update function
    PrimitiveRef update();
    
    // Status queries
    bool isActive() const { return _active; }
    bool isComplete() const;
    int getCurrentCommandIndex() const { return _cmdIndex; }
    int getTotalCommands() const { return _commandString.length(); }
    char getCurrentCommand() const;
    float getElapsedTime() const;
    
    // Debug
    void printStatus() const;
};

#endif // SEQUENCE_EXECUTOR_H