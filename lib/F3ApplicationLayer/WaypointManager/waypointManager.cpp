// waypointManager.cpp
#include "waypointManager.h"

WaypointManager::WaypointManager() 
    : _waypointCount(0), _currentIndex(0), _loopPath(false) {
}

void WaypointManager::clear() {
    _waypointCount = 0;
    _currentIndex = 0;
}

bool WaypointManager::addWaypoint(float x, float y, float theta) {
    if (_waypointCount >= MAX_WAYPOINTS) {
        Serial.println("❌ Waypoint limit reached!");
        return false;
    }
    
    _waypoints[_waypointCount] = {x, y, theta, true};
    _waypointCount++;
    return true;
}

bool WaypointManager::addWaypoint(const TargetPoint& waypoint) {
    if (_waypointCount >= MAX_WAYPOINTS) {
        Serial.println("❌ Waypoint limit reached!");
        return false;
    }
    
    _waypoints[_waypointCount] = waypoint;
    _waypointCount++;
    return true;
}

void WaypointManager::removeLastWaypoint() {
    if (_waypointCount > 0) {
        _waypointCount--;
    }
}

TargetPoint WaypointManager::getCurrentWaypoint() const {
    if (_currentIndex < _waypointCount) {
        return _waypoints[_currentIndex];
    }
    return {0.0f, 0.0f, 0.0f, false};
}

bool WaypointManager::moveToNextWaypoint() {
    if (_currentIndex < _waypointCount - 1) {
        _currentIndex++;
        return true;
    }
    
    // Check if should loop
    if (_loopPath && _waypointCount > 0) {
        _currentIndex = 0;
        return true;
    }
    
    return false;
}

bool WaypointManager::moveToPreviousWaypoint() {
    if (_currentIndex > 0) {
        _currentIndex--;
        return true;
    }
    return false;
}

void WaypointManager::reset() {
    _currentIndex = 0;
}

// === PATH GENERATION FUNCTIONS ===

void WaypointManager::createStraightPath(float distance) {
    clear();
    addWaypoint(distance, 0.0f, 0.0f);
    Serial.print("✓ Straight path created: ");
    Serial.print(distance, 2);
    Serial.println("m");
}

void WaypointManager::createLShapePath(float leg1, float leg2) {
    clear();
    addWaypoint(leg1, 0.0f, 0.0f);        // Đi thẳng leg1
    addWaypoint(leg1, leg2, M_PI/2.0f);   // Rẽ 90° và đi leg2
    Serial.print("✓ L-shape path created: ");
    Serial.print(leg1, 2);
    Serial.print("m + ");
    Serial.print(leg2, 2);
    Serial.println("m");
}

void WaypointManager::createSquarePath(float sideLength) {
    clear();
    addWaypoint(sideLength, 0.0f, 0.0f);
    addWaypoint(sideLength, sideLength, M_PI/2.0f);
    addWaypoint(0.0f, sideLength, M_PI);
    addWaypoint(0.0f, 0.0f, -M_PI/2.0f);
    Serial.print("✓ Square path created: ");
    Serial.print(sideLength, 2);
    Serial.println("m sides");
}

void WaypointManager::createRectanglePath(float width, float height) {
    clear();
    addWaypoint(width, 0.0f, 0.0f);
    addWaypoint(width, height, M_PI/2.0f);
    addWaypoint(0.0f, height, M_PI);
    addWaypoint(0.0f, 0.0f, -M_PI/2.0f);
    Serial.print("✓ Rectangle path created: ");
    Serial.print(width, 2);
    Serial.print("m x ");
    Serial.print(height, 2);
    Serial.println("m");
}

void WaypointManager::createCirclePath(float radius, int segments) {
    clear();
    
    if (segments < 4) segments = 4;
    if (segments > MAX_WAYPOINTS) segments = MAX_WAYPOINTS;
    
    float angleStep = 2.0f * M_PI / segments;
    
    for (int i = 0; i < segments; i++) {
        float angle = i * angleStep;
        float x = radius * (1.0f - cos(angle));
        float y = radius * sin(angle);
        float heading = angle + M_PI/2.0f;
        addWaypoint(x, y, heading);
    }
    
    Serial.print("✓ Circle path created: radius=");
    Serial.print(radius, 2);
    Serial.print("m, segments=");
    Serial.println(segments);
}

// === DEBUG FUNCTIONS ===

void WaypointManager::printWaypoints() const {
    if (_waypointCount == 0) {
        Serial.println("No waypoints");
        return;
    }
    
    Serial.println("\n=== WAYPOINT LIST ===");
    for (int i = 0; i < _waypointCount; i++) {
        Serial.print(i == _currentIndex ? "→ " : "  ");
        Serial.print("[");
        Serial.print(i);
        Serial.print("] X=");
        Serial.print(_waypoints[i].x, 3);
        Serial.print("m, Y=");
        Serial.print(_waypoints[i].y, 3);
        Serial.print("m, θ=");
        Serial.print(_waypoints[i].theta * 180.0f / M_PI, 1);
        Serial.println("°");
    }
    Serial.print("Total: ");
    Serial.print(_waypointCount);
    Serial.print(" waypoints, Current: ");
    Serial.println(_currentIndex);
    Serial.println("=====================\n");
}

void WaypointManager::printCurrentWaypoint() const {
    if (isComplete()) {
        Serial.println("✓ All waypoints completed");
        return;
    }
    
    TargetPoint current = getCurrentWaypoint();
    Serial.print("Current waypoint [");
    Serial.print(_currentIndex);
    Serial.print("/");
    Serial.print(_waypointCount - 1);
    Serial.print("]: (");
    Serial.print(current.x, 2);
    Serial.print(", ");
    Serial.print(current.y, 2);
    Serial.print(", ");
    Serial.print(current.theta * 180.0f / M_PI, 1);
    Serial.println("°)");
}