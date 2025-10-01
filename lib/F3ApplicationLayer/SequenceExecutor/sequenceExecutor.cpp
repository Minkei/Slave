// sequenceExecutor.cpp
#include "sequenceExecutor.h"

SequenceExecutor::SequenceExecutor()
    : _cmdIndex(0), _cmdDuration(3.0f), _startTime(0), _active(false),
      _cumulativeX(0.0f), _cumulativeY(0.0f), _cumulativeTheta(0.0f)
{
}

void SequenceExecutor::setCommandString(String commands, float duration)
{
    _commandString = commands;
    _commandString.toUpperCase();
    _cmdDuration = duration;
}

void SequenceExecutor::start()
{
    _active = true;
    _cmdIndex = 0;
    _startTime = millis();
    _cumulativeX = 0.0f;
    _cumulativeY = 0.0f;
    _cumulativeTheta = 0.0f;

    Serial.print("Sequence started: \"");
    Serial.print(_commandString);
    Serial.print("\" (");
    Serial.print(_commandString.length());
    Serial.println(" commands)");
}

void SequenceExecutor::stop()
{
    _active = false;
    Serial.println("Sequence stopped");
}

void SequenceExecutor::reset()
{
    _cmdIndex = 0;
    _startTime = millis();
    _cumulativeX = 0.0f;
    _cumulativeY = 0.0f;
    _cumulativeTheta = 0.0f;
    _active = false;
}

PrimitiveRef SequenceExecutor::update()
{
    if (!_active || isComplete())
    {
        return {_cumulativeX, _cumulativeY, _cumulativeTheta};
    }

    float currentTime = (millis() - _startTime) / 1000.0f;
    float localTime = currentTime - (_cmdIndex * _cmdDuration);

    // ❌ XÓA PHẦN NÀY - KHÔNG TỰ ĐỘNG CHUYỂN COMMAND NỮA!
    // if (localTime >= _cmdDuration) {
    //     updateCumulativePosition(cmd);
    //     _cmdIndex++;
    //     ...
    // }

    // ✅ CHỈ TẠO REFERENCE, KHÔNG KIỂM TRA TIME!
    // Get current primitive reference (local coordinates)
    PrimitiveRef localRef;
    char cmd = getCurrentCommand();

    switch (cmd)
    {
    case 'F':
        localRef = primitiveForward(localTime);
        break;
    case 'L':
        localRef = primitiveLeft(localTime);
        break;
    case 'R':
        localRef = primitiveRight(localTime);
        break;
    default:
        localRef = {0.0f, 0.0f, 0.0f};
        break;
    }

    // Transform to global coordinates
    return transformToGlobal(localRef);
}

bool SequenceExecutor::forceNextCommand()
{
    if (!_active || isComplete())
    {
        return false;
    }

    // Update cumulative position for the completed command
    char cmd = getCurrentCommand();
    updateCumulativePosition(cmd);

    // Move to next command
    _cmdIndex++;

    // Reset timer (vẫn giữ để tracking, nhưng không dùng cho điều kiện)
    _startTime = millis();

    if (isComplete())
    {
        _active = false;
        Serial.println("✓ Sequence: All commands completed!");
        return false;
    }

    Serial.print("→ Sequence: Moving to command [");
    Serial.print(_cmdIndex);
    Serial.print("]: ");
    Serial.println(getCurrentCommand());

    return true;
}

PrimitiveRef SequenceExecutor::getFinalTarget() const
{
    if (!_active || isComplete())
    {
        return {_cumulativeX, _cumulativeY, _cumulativeTheta};
    }

    char cmd = getCurrentCommand();
    PrimitiveRef finalLocal;

    // Get final position của mỗi primitive (t = _cmdDuration)
    switch (cmd)
    {
    case 'F':
        finalLocal = {0.2f, 0.0f, 0.0f}; // 20cm forward
        break;
    case 'L':
        finalLocal = {0.1f, 0.1f, M_PI / 2.0f}; // L-turn end position
        break;
    case 'R':
        finalLocal = {0.1f, -0.1f, -M_PI / 2.0f}; // R-turn end position
        break;
    default:
        finalLocal = {0.0f, 0.0f, 0.0f};
        break;
    }

    // Transform to global coordinates
    return transformToGlobal(finalLocal);
}

// === PRIMITIVE GENERATORS ===

PrimitiveRef SequenceExecutor::primitiveForward(float t)
{
    // Forward 200mm in 3 seconds (linear interpolation)
    float startX = 0.0f, startY = 0.0f, startTheta = 0.0f;
    float endX = 0.2f, endY = 0.0f, endTheta = 0.0f;

    float alpha;
    if (t <= _cmdDuration)
    {
        alpha = t / _cmdDuration;
    }
    else
    {
        alpha = 1.0f;
    }

    PrimitiveRef ref;
    ref.x = startX + alpha * (endX - startX);
    ref.y = startY + alpha * (endY - startY);
    ref.theta = startTheta + alpha * (endTheta - startTheta);

    return ref;
}

PrimitiveRef SequenceExecutor::primitiveLeft(float t)
{
    // Left turn: 3 phases in 3 seconds
    float t1 = 1.0f;
    float t2 = 2.0f;
    float t3 = 3.0f;

    PrimitiveRef ref;

    if (t <= t1)
    {
        // Phase 1: Straight 50mm
        float alpha = t / t1;
        ref.x = 0.05f * alpha;
        ref.y = 0.0f;
        ref.theta = 0.0f;
    }
    else if (t <= t2)
    {
        // Phase 2: Arc turn 90° (radius 50mm)
        float alpha = (t - t1) / (t2 - t1);
        float angle = alpha * (M_PI / 2.0f);
        ref.x = 0.05f + 0.05f * cos(-M_PI / 2.0f + angle);
        ref.y = 0.05f + 0.05f * sin(-M_PI / 2.0f + angle);
        ref.theta = angle;
    }
    else if (t <= t3)
    {
        // Phase 3: Straight 50mm
        float alpha = (t - t2) / (t3 - t2);
        ref.x = 0.1f;
        ref.y = 0.05f + 0.05f * alpha;
        ref.theta = M_PI / 2.0f;
    }
    else
    {
        // Final position
        ref.x = 0.1f;
        ref.y = 0.1f;
        ref.theta = M_PI / 2.0f;
    }

    return ref;
}

PrimitiveRef SequenceExecutor::primitiveRight(float t)
{
    // Right turn: 3 phases in 3 seconds
    float t1 = 1.0f;
    float t2 = 2.0f;
    float t3 = 3.0f;

    PrimitiveRef ref;

    if (t <= t1)
    {
        // Phase 1: Straight 50mm
        float alpha = t / t1;
        ref.x = 0.05f * alpha;
        ref.y = 0.0f;
        ref.theta = 0.0f;
    }
    else if (t <= t2)
    {
        // Phase 2: Arc turn -90° (radius 50mm)
        float alpha = (t - t1) / (t2 - t1);
        float angle = alpha * (-M_PI / 2.0f);
        ref.x = 0.05f + 0.05f * cos(M_PI / 2.0f + angle);
        ref.y = -0.05f + 0.05f * sin(M_PI / 2.0f + angle);
        ref.theta = angle;
    }
    else if (t <= t3)
    {
        // Phase 3: Straight 50mm
        float alpha = (t - t2) / (t3 - t2);
        ref.x = 0.1f;
        ref.y = -0.05f - 0.05f * alpha;
        ref.theta = -M_PI / 2.0f;
    }
    else
    {
        // Final position
        ref.x = 0.1f;
        ref.y = -0.1f;
        ref.theta = -M_PI / 2.0f;
    }

    return ref;
}

// === HELPER FUNCTIONS ===

void SequenceExecutor::updateCumulativePosition(char cmd)
{
    switch (cmd)
    {
    case 'F':
        _cumulativeX += 0.2f * cos(_cumulativeTheta);
        _cumulativeY += 0.2f * sin(_cumulativeTheta);
        break;

    case 'L':
        _cumulativeX += 0.1f * cos(_cumulativeTheta) - 0.1f * sin(_cumulativeTheta);
        _cumulativeY += 0.1f * sin(_cumulativeTheta) + 0.1f * cos(_cumulativeTheta);
        _cumulativeTheta += M_PI / 2.0f;
        break;

    case 'R':
        _cumulativeX += 0.1f * cos(_cumulativeTheta) + 0.1f * sin(_cumulativeTheta);
        _cumulativeY += 0.1f * sin(_cumulativeTheta) - 0.1f * cos(_cumulativeTheta);
        _cumulativeTheta -= M_PI / 2.0f;
        break;
    }

    // Normalize theta
    while (_cumulativeTheta > M_PI)
        _cumulativeTheta -= 2.0f * M_PI;
    while (_cumulativeTheta < -M_PI)
        _cumulativeTheta += 2.0f * M_PI;
}

PrimitiveRef SequenceExecutor::transformToGlobal(const PrimitiveRef &local) const
{
    float cosTheta = cos(_cumulativeTheta);
    float sinTheta = sin(_cumulativeTheta);

    PrimitiveRef global;
    global.x = _cumulativeX + local.x * cosTheta - local.y * sinTheta;
    global.y = _cumulativeY + local.x * sinTheta + local.y * cosTheta;
    global.theta = _cumulativeTheta + local.theta;

    // Normalize theta
    while (global.theta > M_PI)
        global.theta -= 2.0f * M_PI;
    while (global.theta < -M_PI)
        global.theta += 2.0f * M_PI;

    return global;
}

bool SequenceExecutor::isComplete() const
{
    return _cmdIndex >= _commandString.length();
}

char SequenceExecutor::getCurrentCommand() const
{
    if (_cmdIndex < _commandString.length())
    {
        return _commandString.charAt(_cmdIndex);
    }
    return '\0';
}

float SequenceExecutor::getElapsedTime() const
{
    if (!_active)
        return 0.0f;
    return (millis() - _startTime) / 1000.0f;
}

void SequenceExecutor::printStatus() const
{
    Serial.println("\n=== SEQUENCE STATUS ===");
    Serial.print("Commands: \"");
    Serial.print(_commandString);
    Serial.println("\"");
    Serial.print("Progress: [");
    Serial.print(_cmdIndex);
    Serial.print("/");
    Serial.print(_commandString.length());
    Serial.println("]");
    Serial.print("Current: ");
    Serial.println(getCurrentCommand());
    Serial.print("Active: ");
    Serial.println(_active ? "YES" : "NO");
    Serial.print("Cumulative pose: (");
    Serial.print(_cumulativeX, 3);
    Serial.print(", ");
    Serial.print(_cumulativeY, 3);
    Serial.print(", ");
    Serial.print(_cumulativeTheta * 180.0f / M_PI, 1);
    Serial.println("°)");
    Serial.println("=======================\n");
}