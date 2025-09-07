#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "../Kinematics/kinematics.h"
#include "../../F1HardwareLayer/Encoder/encoder.h"

struct Pose2D {
    float x;
    float y;
    float theta;
};

struct Twist2D {
    float linear_x;
    float linear_y;
    float angular_z;
};

class Odometry {
private:
    Pose2D _current_pose;
    Twist2D _current_twist;
    EncoderMode _encoder_mode;

    long _prev_encoder_left;
    long _prev_encoder_right;

    unsigned long _prev_time_ms;

    Kinematics* _kinematics;

    float _encoder_resolution;
    float _reducer_ratio;

    bool _use_runge_kutta;
    mutable uint8_t _printCounter = 0;

public:
    Odometry(Kinematics* kinematics, float encoder_resolution, float reducer_ratio, EncoderMode encoder_mode = X4_MODE,
            float initial_x = 0.0f, float initial_y = 0.0f, float initial_theta = 0.0f);

    void update(long encoder_left, long encoder_right);

    void updateFromVelocity(float velocity_left, float velocity_right, float dt);

    void updateFromRobotVelocity(const RobotVelocity& velocity, float dt);

    void resetPose(float x = 0.0f, float y = 0.0f, float theta = 0.0f);

    void setPose(const Pose2D& pose);
    
    Pose2D getPose() const { return _current_pose; }
    Twist2D getTwist() const { return _current_twist; }
    
    void getPosition(float& x, float& y) const { x = _current_pose.x; y = _current_pose.y; }
    float getOrientation() const { return _current_pose.theta; }

    float getDistanceFromOrigin() const;
    float getTotalDistance() const { return _total_distance; }
    void setIntergrationMethod(bool use_rk4) { _use_runge_kutta = use_rk4; }
    void printStatus(bool active, uint8_t print_interval) const;

    static float normalizeAngle(float angle);

private:
    float _total_distance;
    void intergrateEuler(float linear_vel, float angular_vel, float dt);
    void intergrateRungeKutta(float linear_vel, float angular_vel, float dt);

};

#endif // ODOMETRY_H