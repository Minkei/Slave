#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "../DifferentialDrive/differentialDrive.h"
#include "../Odometry/odometry.h"

enum MotionStatus {
    MOTION_IDLE,
    MOTION_EXECUTING,
    MOTION_COMPLETED,
    MOTION_ERROR
};

class MotionController {
private:
    // Core components
    DifferentialDrive* _robot;
    Odometry* _odometry;
    
    // Motion parameters
    float _grid_size;           // mm per grid unit
    float _forward_velocity;    // m/s
    float _turn_velocity;       // m/s
    float _turn_angular_vel;    // rad/s
    
    // Tolerances
    float _position_tolerance;  // m
    float _angle_tolerance;     // rad
    
    // State tracking
    MotionStatus _status;
    Pose2D _start_pose;
    unsigned long _start_time;
    unsigned long _timeout_ms;
    char _current_command;
    
    // Callback
    void (*_motion_completed_callback)(char command);
    
public:
    MotionController(DifferentialDrive* robot, Odometry* odometry, float grid_size = 200.0f);
    
    // Motion commands
    bool executeForward();
    bool executeTurnLeft();
    bool executeTurnRight();
    void stop();
    
    // Main update function
    void update();
    
    // Configuration
    void setVelocities(float forward_vel, float turn_vel, float turn_angular);
    void setTolerances(float pos_tol, float ang_tol);
    void setTimeout(unsigned long timeout_ms);
    
    // Status
    MotionStatus getStatus() const;
    bool isExecuting() const;
    bool isCompleted() const;
    
    // Callback
    void setMotionCompletedCallback(void (*callback)(char));
    
    // Debug
    void printStatus() const;
    
private:
    // Motion execution
    bool startMotion(char command);
    void completeMotion();
    void errorMotion(const char* error_msg);
    
    // Progress checking
    float getDistanceTraveled() const;
    float getAngleTurned() const;
    bool checkForwardCompletion() const;
    bool checkTurnCompletion() const;
    bool checkTimeout() const;
};

#endif // MOTION_CONTROLLER_H