#include "purePursuit.h"

// Constructor
PurePursuitController::PurePursuitController(const PurePursuitParams& params) 
    : _params(params), _state(PP_IDLE), _distance_to_target(0.0f), 
      _angle_to_target(0.0f), _heading_error(0.0f), _last_update_time(0), 
      _goal_reached(false), _print_counter(0), _start_time(0),
      _max_distance_from_start(0.0f), _sin_alpha(0.0f), _cos_alpha(1.0f),
      _last_alpha(0.0f), _cache_valid(false) {
    
    _current_target = {0.0f, 0.0f, 0.0f, false};
    _output_velocity = {0.0f, 0.0f};
    _start_pose = {0.0f, 0.0f, 0.0f};
}

// Default parameters based on MATLAB code + improvements
PurePursuitParams PurePursuitController::getDefaultParams() {
    PurePursuitParams params;
    params.max_speed = 0.06f;               
    params.min_speed = 0.02f;               
    params.lookahead_dist = 0.03f;          
    params.goal_tolerance = 0.001f;          
    params.max_angular_vel = 1.0f;          
    params.speed_reduction_factor = 1.5f;   
    
    // Adaptive lookahead defaults
    params.enable_adaptive_lookahead = false;
    params.min_lookahead = 0.02f;           
    params.max_lookahead = 0.08f;           
    params.lookahead_speed_factor = 0.5f;   
    
    // Safety defaults - RELAXED PARAMETERS
    params.max_distance_error = 1.0f;       // 1m max error (thay vì 0.5m)
    params.timeout_ms = 60000;              // 60s timeout (thay vì 30s)
    params.enable_collision_check = false;
    
    // Performance defaults
    params.enable_fast_math = true;
    
    return params;
}

// Main update function - core Pure Pursuit logic
RobotVelocity PurePursuitController::update(const Pose2D& current_pose, const TargetPoint& target) {
    _last_update_time = millis();
    
    // Update target if provided
    if (target.valid) {
        setTarget(target);
    }
    
    // Check if we have a valid target
    if (!_current_target.valid) {
        _state = PP_ERROR;
        _output_velocity = {0.0f, 0.0f};
        return _output_velocity;
    }
    
    // Safety checks
    if (!checkSafetyConditions(current_pose)) {
        _output_velocity = {0.0f, 0.0f};
        return _output_velocity;
    }
    
    // Calculate distance and components once
    float dx = _current_target.x - current_pose.x;
    float dy = _current_target.y - current_pose.y;
    
    if (_params.enable_fast_math) {
        _distance_to_target = fastSqrt(dx * dx + dy * dy);
    } else {
        _distance_to_target = sqrt(dx * dx + dy * dy);
    }
    
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
    _output_velocity = computePurePursuitVelocity(current_pose, dx, dy, _distance_to_target);
    
    return _output_velocity;
}

// Core Pure Pursuit computation - optimized version
RobotVelocity PurePursuitController::computePurePursuitVelocity(
    const Pose2D& current_pose, float dx, float dy, float distance) {
    
    static RobotVelocity velocity; // Static to avoid repeated allocation
    velocity = {0.0f, 0.0f};
    
    // Avoid division by zero
    if (distance < 0.001f) return velocity;
    
    // Calculate angle to target
    float angle_to_target;
    if (distance < _params.goal_tolerance) {
        angle_to_target = _current_target.theta;
    } else {
        if (_params.enable_fast_math) {
            angle_to_target = fastAtan2(dy, dx);
        } else {
            angle_to_target = atan2(dy, dx);
        }
    }
    
    // Calculate heading error with fast normalization
    float alpha;
    if (_params.enable_fast_math) {
        alpha = normalizeAngleFast(angle_to_target - current_pose.theta);
    } else {
        alpha = normalizeAngle(angle_to_target - current_pose.theta);
    }
    
    _heading_error = alpha;
    _angle_to_target = angle_to_target;
    
    // Cache sin/cos if angle hasn't changed much (for performance)
    if (!_cache_valid || abs(alpha - _last_alpha) > 0.01f) {
        if (_params.enable_fast_math) {
            _sin_alpha = fastSin(alpha);
        } else {
            _sin_alpha = sin(alpha);
        }
        _cos_alpha = cos(alpha);
        _last_alpha = alpha;
        _cache_valid = true;
    }
    
    // Pure pursuit law
    if (distance > _params.goal_tolerance) {
        // Calculate adaptive lookahead distance
        float current_speed = sqrt(_output_velocity.linear * _output_velocity.linear);
        float lookahead_distance = calculateAdaptiveLookahead(current_speed);
        lookahead_distance = max(lookahead_distance, distance);
        
        // Calculate curvature using cached sin value
        float curvature = 2.0f * _sin_alpha / lookahead_distance;
        
        // Linear velocity - slower for sharp turns (optimized calculation)
        float alpha_abs = abs(alpha);
        float speed_factor = 1.0f - _params.speed_reduction_factor * alpha_abs * 0.31831f; // 1/π
        speed_factor = max(speed_factor, _params.min_speed / _params.max_speed);
        float linear_vel = _params.max_speed * speed_factor;
        
        // Angular velocity
        float angular_vel = curvature * linear_vel;
        
        // Apply limits
        angular_vel = constrain(angular_vel, -_params.max_angular_vel, _params.max_angular_vel);
        
        velocity.linear = linear_vel;
        velocity.angular = angular_vel;
    }
    
    return velocity;
}

// Adaptive lookahead calculation
float PurePursuitController::calculateAdaptiveLookahead(float current_speed) const {
    if (!_params.enable_adaptive_lookahead) {
        return _params.lookahead_dist;
    }
    
    // Lookahead distance tăng theo tốc độ
    float adaptive_distance = _params.lookahead_dist + 
                             current_speed * _params.lookahead_speed_factor;
    
    return constrain(adaptive_distance, _params.min_lookahead, _params.max_lookahead);
}

// Safety condition checks
bool PurePursuitController::checkSafetyConditions(const Pose2D& current_pose) {
    // Timeout check
    if (_params.timeout_ms > 0) {
        if (millis() - _start_time > _params.timeout_ms) {
            _state = PP_TIMEOUT;
            Serial.println("PP: Timeout reached!");
            return false;
        }
    }
    
    // Distance error check - CHỈ CHECK KHI ĐANG DI CHUYỂN
    if (_params.max_distance_error > 0 && _state == PP_FOLLOWING) {
        // Chỉ check nếu đã di chuyển được một chút (tránh false positive ngay đầu)
        if (millis() - _start_time > 1000) { // Wait 1 second before checking
            if (_distance_to_target > _params.max_distance_error) {
                _state = PP_SAFETY_STOP;
                Serial.print("PP: Distance error too large! Current: ");
                Serial.print(_distance_to_target, 3);
                Serial.print("m, Max: ");
                Serial.print(_params.max_distance_error, 3);
                Serial.println("m");
                return false;
            }
        }
    }
    
    // Kiểm tra robot có đi quá xa so với vị trí ban đầu không
    if (_max_distance_from_start > 0) {
        float dist_from_start = sqrt((current_pose.x - _start_pose.x) * (current_pose.x - _start_pose.x) + 
                                   (current_pose.y - _start_pose.y) * (current_pose.y - _start_pose.y));
        if (dist_from_start > _max_distance_from_start * 3.0f) { // Tăng từ 2.0f lên 3.0f
            _state = PP_SAFETY_STOP;
            Serial.print("PP: Robot went too far from start! Distance: ");
            Serial.print(dist_from_start, 3);
            Serial.print("m, Max: ");
            Serial.print(_max_distance_from_start * 3.0f, 3);
            Serial.println("m");
            return false;
        }
    }
    
    return true;
}

// Fast math implementations
inline float PurePursuitController::fastSin(float x) {
    // Normalize to [-π, π]
    x = normalizeAngleFast(x);
    
    // Simple polynomial approximation for small angles
    if (abs(x) < 0.5f) {
        return x - (x*x*x)/6.0f;
    }
    return sin(x); // Fall back to standard sin for larger angles
}

inline float PurePursuitController::fastSqrt(float x) {
    if (x <= 0.0f) return 0.0f;  // Add bounds check
    if (x < 0.01f) return 0.0f;
    
    // Newton-Raphson with better initial guess
    float guess = x * 0.5f;
    if (x > 1.0f) guess = x * 0.25f;  // Better for large numbers
    
    for (int i = 0; i < 3; i++) {
        guess = 0.5f * (guess + x / guess);
    }
    return guess;
}

inline float PurePursuitController::fastAtan2(float y, float x) {
    // For small angles, use simple approximation
    if (abs(x) > abs(y) * 3.0f) {
        return y / x;
    }
    return atan2(y, x); // Fall back to standard atan2
}

// Angle normalization functions
float PurePursuitController::normalizeAngle(float angle) {
    // Standard normalization
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float PurePursuitController::normalizeAngleFast(float angle) {
    // Faster than while loops for small deviations
    if (angle > M_PI) {
        angle -= 2.0f * M_PI;
        if (angle > M_PI) angle -= 2.0f * M_PI;  // Handle larger deviations
    } else if (angle < -M_PI) {
        angle += 2.0f * M_PI;
        if (angle < -M_PI) angle += 2.0f * M_PI;
    }
    return angle;
}

// Helper functions
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
    _cache_valid = false; // Invalidate cache when parameters change
}

void PurePursuitController::setTarget(const TargetPoint& target) {
    _current_target = target;
    _goal_reached = false;
    _state = PP_IDLE;
    _start_time = millis();
    _cache_valid = false;
    
    // Tính khoảng cách tối đa có thể từ start position
    _max_distance_from_start = sqrt(target.x * target.x + target.y * target.y) + 0.5f;
}

void PurePursuitController::setTarget(float x, float y, float theta) {
    TargetPoint target = {x, y, theta, true};
    setTarget(target);
}

void PurePursuitController::setTargetWithStartPose(const TargetPoint& target, const Pose2D& start_pose) {
    _start_pose = start_pose;
    setTarget(target);
}

void PurePursuitController::reset() {
    _state = PP_IDLE;
    _goal_reached = false;
    _output_velocity = {0.0f, 0.0f};
    _distance_to_target = 0.0f;
    _angle_to_target = 0.0f;
    _heading_error = 0.0f;
    _cache_valid = false;
    _start_time = millis();
}

void PurePursuitController::emergencyStop() {
    _state = PP_SAFETY_STOP;
    _output_velocity = {0.0f, 0.0f};
    _cache_valid = false;
}

// Debug functions
void PurePursuitController::printStatus(bool active, uint8_t print_interval) const {
    if (!active) return;
    _print_counter++;
    if (_print_counter >= print_interval) {
        Serial.print("PP[");
        Serial.print(getStateString());
        Serial.print("] D:");
        Serial.print(_distance_to_target, 3);
        Serial.print("m E:");
        Serial.print(getHeadingErrorDegrees(), 1);
        Serial.print("° V:");
        Serial.print(_output_velocity.linear, 3);
        Serial.print("/");
        Serial.print(_output_velocity.angular, 3);
        
        if (_state == PP_FOLLOWING) {
            Serial.print(" T:");
            Serial.print(getElapsedTime());
            Serial.print("ms");
        }
        
        Serial.println();
        _print_counter = 0;
    }
}

void PurePursuitController::printDetailedStatus() const {
    Serial.println("=== PURE PURSUIT DETAILED STATUS ===");
    Serial.print("State: ");
    Serial.println(getStateString());
    
    Serial.print("Target: (");
    Serial.print(_current_target.x, 3);
    Serial.print(", ");
    Serial.print(_current_target.y, 3);
    Serial.print(", ");
    Serial.print(_current_target.theta * 180.0f / M_PI, 1);
    Serial.print("°) Valid: ");
    Serial.println(_current_target.valid ? "YES" : "NO");
    
    Serial.print("Distance to target: ");
    Serial.print(_distance_to_target, 3);
    Serial.println(" m");
    
    Serial.print("Heading error: ");
    Serial.print(getHeadingErrorDegrees(), 1);
    Serial.println("°");
    
    Serial.print("Output velocity: Linear=");
    Serial.print(_output_velocity.linear, 3);
    Serial.print(" m/s, Angular=");
    Serial.print(_output_velocity.angular, 3);
    Serial.println(" rad/s");
    
    Serial.print("Elapsed time: ");
    Serial.print(getElapsedTime());
    Serial.println(" ms");
    
    Serial.print("Parameters: MaxSpd=");
    Serial.print(_params.max_speed, 3);
    Serial.print(" MinSpd=");
    Serial.print(_params.min_speed, 3);
    Serial.print(" LookAhd=");
    Serial.print(_params.lookahead_dist, 3);
    Serial.print(" Tol=");
    Serial.println(_params.goal_tolerance, 3);
    
    if (_params.enable_adaptive_lookahead) {
        float current_speed = sqrt(_output_velocity.linear * _output_velocity.linear);
        Serial.print("Adaptive lookahead: ");
        Serial.print(calculateAdaptiveLookahead(current_speed), 3);
        Serial.println(" m");
    }
    
    Serial.print("Fast math: ");
    Serial.println(_params.enable_fast_math ? "ENABLED" : "DISABLED");
}

String PurePursuitController::getStatusString() const {
    String status = "PP[" + getStateString() + "] ";
    status += "D:" + String(_distance_to_target, 3);
    status += " E:" + String(getHeadingErrorDegrees(), 1) + "°";
    return status;
}

String PurePursuitController::getStateString() const {
    switch(_state) {
        case PP_IDLE: return "IDLE";
        case PP_FOLLOWING: return "FOLLOW";
        case PP_GOAL_REACHED: return "GOAL";
        case PP_ERROR: return "ERROR";
        case PP_TIMEOUT: return "TIMEOUT";
        case PP_SAFETY_STOP: return "SAFETY";
        default: return "UNKNOWN";
    }
}