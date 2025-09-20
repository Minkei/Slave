#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <Arduino.h>
#include <math.h>
#include "../Kinematics/kinematics.h"
#include "../Odometry/odometry.h"

// Target point structure
struct TargetPoint {
    float x;        // m
    float y;        // m  
    float theta;    // rad
    bool valid;     // Is this target valid?
};

// Pure Pursuit parameters
struct PurePursuitParams {
    float max_speed;        // m/s
    float min_speed;        // m/s
    float lookahead_dist;   // m
    float goal_tolerance;   // m
    float max_angular_vel;  // rad/s
    float speed_reduction_factor; // How much to slow down for turns
};

// Controller state
enum PurePursuitState {
    PP_IDLE,
    PP_FOLLOWING,
    PP_GOAL_REACHED,
    PP_ERROR
};

class PurePursuitController {
private:
    // Parameters
    PurePursuitParams _params;
    
    // Current state
    PurePursuitState _state;
    TargetPoint _current_target;
    RobotVelocity _output_velocity;
    
    // Status tracking
    float _distance_to_target;
    float _angle_to_target;
    float _heading_error;
    unsigned long _last_update_time;
    bool _goal_reached;
    
    // Helper functions
    float normalizeAngle(float angle);
    float calculateDistance(const Pose2D& current, const TargetPoint& target);
    float calculateAngleToTarget(const Pose2D& current, const TargetPoint& target);
    RobotVelocity computePurePursuitVelocity(const Pose2D& current_pose);
    
public:
    // Constructor
    PurePursuitController(const PurePursuitParams& params = getDefaultParams());
    
    // Main control function
    RobotVelocity update(const Pose2D& current_pose, const TargetPoint& target);
    
    // Configuration
    void setParams(const PurePursuitParams& params);
    PurePursuitParams getParams() const { return _params; }
    static PurePursuitParams getDefaultParams();
    
    // Target management
    void setTarget(const TargetPoint& target);
    void setTarget(float x, float y, float theta = 0.0f);
    TargetPoint getCurrentTarget() const { return _current_target; }
    
    // Status queries
    bool isGoalReached() const { return _goal_reached; }
    PurePursuitState getState() const { return _state; }
    float getDistanceToTarget() const { return _distance_to_target; }
    float getHeadingError() const { return _heading_error; }
    RobotVelocity getLastOutput() const { return _output_velocity; }
    
    // Reset controller
    void reset();
    
    // Debug output
    void printStatus(bool active = true, uint8_t print_interval = 20) const;
    String getStatusString() const;
    
private:
    mutable uint8_t _print_counter;
};

#endif // PURE_PURSUIT_CONTROLLER_H