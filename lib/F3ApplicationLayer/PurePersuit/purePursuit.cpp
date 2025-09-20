#include "purePursuit.h"

// Constructor
PurePursuitController::PurePursuitController(const PurePursuitParams& params) 
    : _params(params), _state(PP_IDLE), _distance_to_target(0.0f), 
      _angle_to_target(0.0f), _heading_error(0.0f), _last_update_time(0), 
      _goal_reached(false), _print_counter(0) {
    
    _current_target = {0.0f, 0.0f, 0.0f, false};
    _output_velocity = {0.0f, 0.0f};
}

// Default parameters based on MATLAB code
PurePursuitParams PurePursuitController::getDefaultParams() {
    PurePursuitParams params;
    params.max_speed = 0.06f;               // 60mm/s from MATLAB
    params.min_speed = 0.02f;               // 20mm/s from MATLAB  
    params.lookahead_dist = 0.03f;          // 30mm from MATLAB
    params.goal_tolerance = 0.01f;          // 10mm from MATLAB
    params.max_angular_vel = 1.0f;          // 1 rad/s from MATLAB
    params.speed_reduction_factor = 1.5f;   // From MATLAB: 1.5*abs(alpha)/pi
    return params;
}

// Main update function - core Pure Pursuit logic
RobotVelocity PurePursuitController::update(const Pose2D& current_pose, const TargetPoint& target) {
    _last_update_time = millis();
    
    // Update target if provided
    if (target.valid) {
        _current_target = target;
    }
    
    // Check if we have a valid target
    if (!_current_target.valid) {
        _state = PP_ERROR;
        _output_velocity = {0.0f, 0.0f};
        return _output_velocity;
    }
    
    // Calculate distance and angle to target
    _distance_to_target = calculateDistance(current_pose, _current_target);
    
    // Check if goal is reached
    if (_distance_to_target < _params.goal_tolerance) {
        _state = PP_GOAL_REACHED;
        _goal_reached = true;
        _output_velocity = {0.0f, 0.0f};
        return _output_velocity;
    }
    
    // Follow target using Pure Pursuit
    _state = PP_FOLLOWING;
    _goal_reached = false;
    _output_velocity = computePurePursuitVelocity(current_pose);
    
    return _output_velocity;
}

// Core Pure Pursuit computation - direct translation from MATLAB
RobotVelocity PurePursuitController::computePurePursuitVelocity(const Pose2D& current_pose) {
    RobotVelocity velocity = {0.0f, 0.0f};
    
    // Extract current position (same as MATLAB)
    float x = current_pose.x;
    float y = current_pose.y; 
    float theta = current_pose.theta;
    
    // Target point from trajectory (same as MATLAB)
    float target_x = _current_target.x;
    float target_y = _current_target.y;
    float target_theta = _current_target.theta;
    
    // Calculate distance and angle to target (same as MATLAB)
    float dx = target_x - x;
    float dy = target_y - y;
    float distance = sqrt(dx * dx + dy * dy);
    
    // If very close to target, use target heading (same as MATLAB)
    float angle_to_target;
    if (distance < _params.goal_tolerance) {
        angle_to_target = target_theta;
    } else {
        angle_to_target = atan2(dy, dx);
    }
    
    // Calculate heading error (same as MATLAB)
    float alpha = angle_to_target - theta;
    alpha = normalizeAngle(alpha);  // Same normalization logic as MATLAB
    _heading_error = alpha;
    _angle_to_target = angle_to_target;
    
    // Pure pursuit law (same as MATLAB)
    if (distance > _params.goal_tolerance) {
        // Calculate curvature (same as MATLAB)
        float curvature = 2.0f * sin(alpha) / _params.lookahead_dist;
        
        // Linear velocity - slower for sharp turns (same as MATLAB)
        float linear_vel = _params.max_speed * (1.0f - _params.speed_reduction_factor * abs(alpha) / M_PI);
        linear_vel = max(linear_vel, _params.min_speed);
        
        // Angular velocity (same as MATLAB)
        float angular_vel = curvature * linear_vel;
        
        // Apply limits (same as MATLAB)
        angular_vel = constrain(angular_vel, -_params.max_angular_vel, _params.max_angular_vel);
        
        velocity.linear = linear_vel;
        velocity.angular = angular_vel;
    } else {
        // At target - minimal movement (same as MATLAB)
        velocity.linear = 0.0f;
        velocity.angular = 0.0f;
    }
    
    return velocity;
}

// Helper functions
float PurePursuitController::normalizeAngle(float angle) {
    // Same logic as MATLAB: while alpha > pi, alpha = alpha - 2*pi; end
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float PurePursuitController::calculateDistance(const Pose2D& current, const TargetPoint& target) {
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    return sqrt(dx * dx + dy * dy);
}

float PurePursuitController::calculateAngleToTarget(const Pose2D& current, const TargetPoint& target) {
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    return atan2(dy, dx);
}

// Configuration functions
void PurePursuitController::setParams(const PurePursuitParams& params) {
    _params = params;
}

void PurePursuitController::setTarget(const TargetPoint& target) {
    _current_target = target;
    _goal_reached = false;
    _state = PP_IDLE;
}

void PurePursuitController::setTarget(float x, float y, float theta) {
    _current_target = {x, y, theta, true};
    _goal_reached = false;
    _state = PP_IDLE;
}

void PurePursuitController::reset() {
    _state = PP_IDLE;
    _goal_reached = false;
    _output_velocity = {0.0f, 0.0f};
    _distance_to_target = 0.0f;
    _angle_to_target = 0.0f;
    _heading_error = 0.0f;
}

// Debug functions
void PurePursuitController::printStatus(bool active, uint8_t print_interval) const {
    if (!active) return;
    _print_counter++;
    if (_print_counter == print_interval) {
        Serial.print("PP - State:");
        switch(_state) {
            case PP_IDLE: Serial.print("IDLE"); break;
            case PP_FOLLOWING: Serial.print("FOLLOW"); break; 
            case PP_GOAL_REACHED: Serial.print("GOAL"); break;
            case PP_ERROR: Serial.print("ERROR"); break;
        }
        Serial.print(", Dist:");
        Serial.print(_distance_to_target, 3);
        Serial.print("m, HeadErr:");
        Serial.print(_heading_error * 180.0f / M_PI, 1);
        Serial.print("deg, Vel:");
        Serial.print(_output_velocity.linear, 3);
        Serial.print("/");
        Serial.print(_output_velocity.angular, 3);
        Serial.println();
        _print_counter = 0;
    }
}

String PurePursuitController::getStatusString() const {
    String status = "PP[";
    switch(_state) {
        case PP_IDLE: status += "IDLE"; break;
        case PP_FOLLOWING: status += "FOLLOW"; break;
        case PP_GOAL_REACHED: status += "GOAL"; break; 
        case PP_ERROR: status += "ERROR"; break;
    }
    status += "] D:" + String(_distance_to_target, 3);
    status += " E:" + String(_heading_error * 180.0f / M_PI, 1) + "°";
    return status;
}