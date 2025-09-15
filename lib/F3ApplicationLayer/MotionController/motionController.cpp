#include "motionController.h"

MotionController::MotionController(DifferentialDrive* robot, Odometry* odometry, float grid_size)
    : _robot(robot), _odometry(odometry), _grid_size(grid_size),
      _forward_velocity(0.08f), _turn_velocity(0.06f), _turn_angular_vel(0.6f),
      _position_tolerance(0.01f), _angle_tolerance(0.05f),
      _status(MOTION_IDLE), _start_time(0), _timeout_ms(10000),
      _motion_completed_callback(nullptr), _current_command('\0') {
}

bool MotionController::executeForward() {
    if (_status == MOTION_EXECUTING) {
        return false;
    }
    return startMotion('F');
}

bool MotionController::executeTurnLeft() {
    if (_status == MOTION_EXECUTING) {
        return false;
    }
    return startMotion('L');
}

bool MotionController::executeTurnRight() {
    if (_status == MOTION_EXECUTING) {
        return false;
    }
    return startMotion('R');
}

void MotionController::stop() {
    if (_robot) {
        _robot->setVelocity(0, 0);
    }
    _status = MOTION_IDLE;
    _current_command = '\0';
}

void MotionController::update() {
    if (_status != MOTION_EXECUTING) return;
    
    // Check timeout
    if (checkTimeout()) {
        errorMotion("Timeout");
        return;
    }
    
    // Check completion
    bool completed = false;
    switch (_current_command) {
        case 'F':
            completed = checkForwardCompletion();
            break;
        case 'L':
        case 'R':
            completed = checkTurnCompletion();
            break;
    }
    
    if (completed) {
        completeMotion();
    }
}

void MotionController::setVelocities(float forward_vel, float turn_vel, float turn_angular) {
    _forward_velocity = forward_vel;
    _turn_velocity = turn_vel;
    _turn_angular_vel = turn_angular;
}

void MotionController::setTolerances(float pos_tol, float ang_tol) {
    _position_tolerance = pos_tol;
    _angle_tolerance = ang_tol;
}

void MotionController::setTimeout(unsigned long timeout_ms) {
    _timeout_ms = timeout_ms;
}

MotionStatus MotionController::getStatus() const {
    return _status;
}

bool MotionController::isExecuting() const {
    return _status == MOTION_EXECUTING;
}

bool MotionController::isCompleted() const {
    return _status == MOTION_COMPLETED;
}

void MotionController::setMotionCompletedCallback(void (*callback)(char)) {
    _motion_completed_callback = callback;
}

void MotionController::printStatus() const {
    Serial.print("Motion: ");
    switch (_status) {
        case MOTION_IDLE: Serial.print("IDLE"); break;
        case MOTION_EXECUTING: Serial.print("EXECUTING"); break;
        case MOTION_COMPLETED: Serial.print("COMPLETED"); break;
        case MOTION_ERROR: Serial.print("ERROR"); break;
    }
    
    if (_status == MOTION_EXECUTING) {
        Serial.print(" (");
        Serial.print(_current_command);
        Serial.print(", ");
        Serial.print((millis() - _start_time) / 1000.0f, 1);
        Serial.print("s)");
    }
    Serial.println();
}

// Private methods

bool MotionController::startMotion(char command) {
    if (!_robot || !_odometry) {
        return false;
    }
    
    _start_pose = _odometry->getPose();
    _start_time = millis();
    _current_command = command;
    _status = MOTION_EXECUTING;
    
    switch (command) {
        case 'F':
            _robot->setVelocity(_forward_velocity, 0);
            break;
        case 'L':
            _robot->setVelocity(_turn_velocity, _turn_angular_vel);
            break;
        case 'R':
            _robot->setVelocity(_turn_velocity, -_turn_angular_vel);
            break;
        default:
            _status = MOTION_IDLE;
            return false;
    }
    
    return true;
}

void MotionController::completeMotion() {
    if (_robot) {
        _robot->setVelocity(0, 0);
    }
    
    _status = MOTION_COMPLETED;
    
    if (_motion_completed_callback) {
        _motion_completed_callback(_current_command);
    }
    
    _current_command = '\0';
    _status = MOTION_IDLE;
}

void MotionController::errorMotion(const char* error_msg) {
    if (_robot) {
        _robot->setVelocity(0, 0);
    }
    
    _status = MOTION_ERROR;
    _current_command = '\0';
    
    Serial.print("Motion error: ");
    Serial.println(error_msg);
}

float MotionController::getDistanceTraveled() const {
    if (!_odometry) return 0.0f;
    
    Pose2D current = _odometry->getPose();
    float dx = current.x - _start_pose.x;
    float dy = current.y - _start_pose.y;
    return sqrt(dx * dx + dy * dy);
}

float MotionController::getAngleTurned() const {
    if (!_odometry) return 0.0f;
    
    Pose2D current = _odometry->getPose();
    float angle_diff = current.theta - _start_pose.theta;
    
    // Normalize to [-pi, pi]
    while (angle_diff > PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;
    
    return fabs(angle_diff);
}

bool MotionController::checkForwardCompletion() const {
    float target_distance = _grid_size / 1000.0f;  // mm to m
    float distance_traveled = getDistanceTraveled();
    return distance_traveled >= (target_distance - _position_tolerance);
}

bool MotionController::checkTurnCompletion() const {
    float target_angle = PI / 2.0f;  // 90 degrees
    float angle_turned = getAngleTurned();
    return angle_turned >= (target_angle - _angle_tolerance);
}

bool MotionController::checkTimeout() const {
    return (millis() - _start_time) > _timeout_ms;
}