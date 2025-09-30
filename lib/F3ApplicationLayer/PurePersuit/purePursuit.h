// purePursuit.h
#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <Arduino.h>
#include <math.h>

#include "../Kinematics/kinematics.h"
#include "../Odometry/odometry.h"

struct TargetPoint
{
    float x;     // m
    float y;     // m
    float theta; // rad
    bool valid;  // Is this target valid?
};

struct PurePursuitParams
{
    // Pure Pursuit params
    float max_speed;              // m/s
    float min_speed;              // m/s
    float lookahead_dist;         // m
    float goal_tolerance;         // m
    float max_angular_vel;        // rad/s
    float speed_reduction_factor; // How much to slow down for turns

    // Adaptive lookahead
    bool enable_adaptive_lookahead;
    float min_lookahead;
    float max_lookahead;
    float lookahead_speed_factor;

    // Safety parameters
    float max_distance_error;    // stop if too far from target
    float timeout_ms;            // timeout if cannot reach target
    bool enable_collision_check; // place holder for collision check

    // Performance parameters
    bool enable_fast_math; // ✅ FIXED: changed from float to bool
};

// Controller state
enum PurePursuitState
{
    PP_IDLE,
    PP_FOLLOWING,
    PP_GOAL_REACHED,
    PP_ERROR,
    PP_TIMEOUT,
    PP_SAFETY_STOP
};

class PurePursuitController
{
private:
    PurePursuitParams _params;
    PurePursuitState _state;
    TargetPoint _current_target;
    RobotVelocity _output_velocity;
    
    float _distance_to_target;
    float _angle_to_target;
    float _heading_error;
    uint32_t _last_update_time;
    bool _goal_reached;
    
    unsigned long _start_time;
    float _max_distance_from_start;
    Pose2D _start_pose;

    // Performance optimization - cached values
    float _sin_alpha;
    float _cos_alpha;
    float _last_alpha;
    bool _cache_valid;

    //Helper functions
    float normalizeAngle(float angle);
    float normalizeAngleFast(float angle);
    float calculateDistance(const Pose2D& current, const TargetPoint& target);
    float calculateAngleToTarget(const Pose2D& current, const TargetPoint& target);
    RobotVelocity computePurePursuitVelocity(const Pose2D& current_pose, float dx, float dy, float distance);

    // Adaptive lookahead
    float calculateAdaptiveLookahead(float current_speed) const;
    
    // Safety checks
    bool checkSafetyConditions(const Pose2D& current_pose);

    // Fast math functions
    inline float fastSin(float x);
    inline float fastSqrt(float x);
    inline float fastAtan2(float y, float x);

public:
    // Constructor
    PurePursuitController(const PurePursuitParams& params = getDefaultParams());
    
    // ✅ FIXED: Main control function - NO LONGER takes target parameter
    RobotVelocity update(const Pose2D& current_pose);

    // Configuration
    void setParams(const PurePursuitParams& params);
    PurePursuitParams getParams() const { return _params; }
    static PurePursuitParams getDefaultParams();
    
    // Target management
    void setTarget(const TargetPoint& target);
    void setTarget(float x, float y, float theta = 0.0f);
    void setTargetWithStartPose(const TargetPoint& target, const Pose2D& start_pose);
    TargetPoint getCurrentTarget() const { return _current_target; }
    
    // Status queries
    bool isGoalReached() const { return _goal_reached; }
    PurePursuitState getState() const { return _state; }
    float getDistanceToTarget() const { return _distance_to_target; }
    float getHeadingError() const { return _heading_error; }
    float getHeadingErrorDegrees() const { return _heading_error * 180.0f / M_PI; }
    RobotVelocity getLastOutput() const { return _output_velocity; }
    
    // Safety status
    bool isTimeout() const { return _state == PP_TIMEOUT; }
    bool isSafetyStopped() const { return _state == PP_SAFETY_STOP; }
    unsigned long getElapsedTime() const { return millis() - _start_time; }
    
    // Reset controller
    void reset();
    void emergencyStop();
    
    // Parameter tuning helpers
    void setMaxSpeed(float speed) { _params.max_speed = speed; }
    void setMinSpeed(float speed) { _params.min_speed = speed; }
    void setLookaheadDistance(float distance) { _params.lookahead_dist = distance; }
    void setGoalTolerance(float tolerance) { _params.goal_tolerance = tolerance; }
    void setSpeedReductionFactor(float factor) { _params.speed_reduction_factor = factor; }
    void setTimeout(float timeout_ms) { _params.timeout_ms = timeout_ms; }
    void enableFastMath(bool enable) { _params.enable_fast_math = enable; _cache_valid = false; }
    void enableAdaptiveLookahead(bool enable) { _params.enable_adaptive_lookahead = enable; }
    
    // Debug output
    void printStatus(bool active = true, uint8_t print_interval = 20) const;
    void printDetailedStatus() const;
    String getStatusString() const;
    String getStateString() const;

private:
    mutable uint8_t _print_counter;
};

#endif // PURE_PURSUIT_H