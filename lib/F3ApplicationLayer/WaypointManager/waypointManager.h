// waypointManager.h
#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <Arduino.h>
#include "../PurePersuit/purePursuit.h"

#define MAX_WAYPOINTS 20

class WaypointManager {
private:
    TargetPoint _waypoints[MAX_WAYPOINTS];
    int _waypointCount;
    int _currentIndex;
    bool _loopPath;
    
public:
    WaypointManager();
    
    // Waypoint management
    void clear();
    bool addWaypoint(float x, float y, float theta = 0.0f);
    bool addWaypoint(const TargetPoint& waypoint);
    void removeLastWaypoint();
    
    // Navigation
    TargetPoint getCurrentWaypoint() const;
    bool moveToNextWaypoint();
    bool moveToPreviousWaypoint();
    void reset();
    
    // Status
    int getWaypointCount() const { return _waypointCount; }
    int getCurrentIndex() const { return _currentIndex; }
    bool isComplete() const { return _currentIndex >= _waypointCount; }
    bool isEmpty() const { return _waypointCount == 0; }
    
    // Path generation helpers
    void createStraightPath(float distance);
    void createLShapePath(float leg1, float leg2);
    void createSquarePath(float sideLength);
    void createRectanglePath(float width, float height);
    void createCirclePath(float radius, int segments = 8);
    
    // Configuration
    void setLoopPath(bool loop) { _loopPath = loop; }
    bool isLooping() const { return _loopPath; }
    
    // Debug
    void printWaypoints() const;
    void printCurrentWaypoint() const;
};

#endif // WAYPOINT_MANAGER_H