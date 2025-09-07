#include "odometry.h"
#include <Arduino.h>
#include <cmath>

Odometry::Odometry(Kinematics *kinematics, float encoder_resolution, float reducer_ratio, EncoderMode encoder_mode, float initial_x, float initial_y, float initial_theta)
    : _kinematics(kinematics), _encoder_resolution(encoder_resolution), _reducer_ratio(reducer_ratio), _encoder_mode(encoder_mode), _use_runge_kutta(false), _total_distance(0.0f)
{
    _current_pose.x = initial_x;
    _current_pose.y = initial_y;
    _current_pose.theta = initial_theta;

    _current_twist.linear_x = 0.0f;
    _current_twist.linear_y = 0.0f;
    _current_twist.angular_z = 0.0f;

    _prev_encoder_left = 0;
    _prev_encoder_right = 0;
    _prev_time_ms = millis();
}

void Odometry::update(long encoder_left, long encoder_right)
{
    unsigned long current_time = millis();
    
    float dt = (float)(current_time - _prev_time_ms) / 1000.0f;
    
    if (dt <= 0.0f) return;

    long delta_left = encoder_left - _prev_encoder_left;
    long delta_right = encoder_right - _prev_encoder_right;

    float ticks_per_wheel_rotation = _encoder_resolution * _reducer_ratio * _encoder_mode;

    float wheel_rotation_left = (float)delta_left / ticks_per_wheel_rotation;
    float wheel_rotation_right = (float)delta_right / ticks_per_wheel_rotation;

    float wheel_radius = _kinematics->getWheelRadius();

    float distance_left = wheel_rotation_left * 2.0f * M_PI * wheel_radius;
    float distance_right = wheel_rotation_right * 2.0f * M_PI * wheel_radius;

    float velocity_left = distance_left / dt;
    float velocity_right = distance_right / dt;

    updateFromVelocity(velocity_left, velocity_right, dt);

    _prev_encoder_left = encoder_left;
    _prev_encoder_right = encoder_right;
    _prev_time_ms = current_time;
}

void Odometry::updateFromVelocity(float velocity_left, float velocity_right, float dt)
{
    WheelRPM wheel_velocities_as_rpm;
    
    wheel_velocities_as_rpm.left = _kinematics->velocityToRPM(velocity_left);
    wheel_velocities_as_rpm.right = _kinematics->velocityToRPM(velocity_right);

    RobotVelocity robot_vel = _kinematics->forwardKinematics(wheel_velocities_as_rpm);

    updateFromRobotVelocity(robot_vel, dt);
}

void Odometry::updateFromRobotVelocity(const RobotVelocity &velocity, float dt)
{
    _current_twist.linear_x = velocity.linear;
    _current_twist.linear_y = 0.0f;
    _current_twist.angular_z = velocity.angular;

    if (_use_runge_kutta) {
        intergrateRungeKutta(velocity.linear, velocity.angular, dt);
    } else {
        intergrateEuler(velocity.linear, velocity.angular, dt);
    }

    _total_distance += fabs(velocity.linear) * dt;

    _current_pose.theta = normalizeAngle(_current_pose.theta);
}

void Odometry::resetPose(float x, float y, float theta)
{
    _current_pose.x = x;
    _current_pose.y = y;
    _current_pose.theta = normalizeAngle(theta);
    _total_distance = 0.0f;

    _current_twist.linear_x = 0.0f;
    _current_twist.linear_y = 0.0f;
    _current_twist.angular_z = 0.0f;
}

void Odometry::setPose(const Pose2D &pose)
{
    _current_pose = pose;
    _current_pose.theta = normalizeAngle(_current_pose.theta);
}

float Odometry::getDistanceFromOrigin() const
{
    return sqrt(_current_pose.x * _current_pose.x + _current_pose.y * _current_pose.y);
}

void Odometry::printStatus(bool active, uint8_t print_interval) const
{
    if (!active) return;
    _printCounter++;
    if (_printCounter == print_interval) {
        Serial.print("X:");
        Serial.print(_current_pose.x);
        Serial.print(",");
        Serial.print("Y:");
        Serial.print(_current_pose.y);
        Serial.print(",");
        Serial.print("THETA:");
        Serial.print(_current_pose.theta);
        Serial.print(",");
        Serial.print("DIST:");
        Serial.print(getDistanceFromOrigin());
        Serial.print(",");
        Serial.print("DIST:");
        Serial.print(_total_distance);
        Serial.println();
        _printCounter = 0;
    }
}

float Odometry::normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void Odometry::intergrateEuler(float linear_vel, float angular_vel, float dt)
{
    float cos_theta = cos(_current_pose.theta);
    float sin_theta = sin(_current_pose.theta);

    _current_pose.x += linear_vel * cos_theta * dt;
    _current_pose.y += linear_vel * sin_theta * dt;
    _current_pose.theta += angular_vel * dt;
}

void Odometry::intergrateRungeKutta(float linear_vel, float angular_vel, float dt)
{
    if(fabs(angular_vel) < 1e-6) {
        intergrateEuler(linear_vel, angular_vel, dt);
        return;
    }

    float R = linear_vel / angular_vel;

    float theta0 = _current_pose.theta;
    float dtheta = angular_vel * dt;

    float sin_dtheta = sin(dtheta);
    float cos_dtheta = cos(dtheta);

    float dx = R * sin_dtheta;
    float dy = R * (1.0f - cos_dtheta);

    float cos_theta0 = cos(theta0);
    float sin_theta0 = sin(theta0);

    _current_pose.x += dx * cos_theta0 - dy * sin_theta0;
    _current_pose.y += dx * sin_theta0 + dy * cos_theta0;
    _current_pose.theta += dtheta;
}
