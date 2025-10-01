#include "kinematics.h"
#include <cmath>
#include <algorithm>

Kinematics::Kinematics(float wheel_radius, float wheel_base)
    : _wheel_radius(wheel_radius), _wheel_base(wheel_base) {
}

RobotVelocity Kinematics::forwardKinematics(const WheelRPM &wheel_rpm) const
{
    // Convert RPM to velocity
    float v_left = rpmToVelocity(wheel_rpm.left);
    float v_right = rpmToVelocity(wheel_rpm.right);
    
    // Calculate linear and angular velocity
    RobotVelocity robot_vel;
    robot_vel.linear = (v_left + v_right) / 2.0f;
    robot_vel.angular = (v_right - v_left) / _wheel_base;
    return robot_vel;
}

WheelRPM Kinematics::inverseKinematics(const RobotVelocity &robot_vel) const
{
    float v_left = robot_vel.linear - (robot_vel.angular * _wheel_base / 2.0f);
    float v_right = robot_vel.linear + (robot_vel.angular * _wheel_base / 2.0f);
    
    WheelRPM wheel_rpm;
    wheel_rpm.left = velocityToRPM(v_left);
    wheel_rpm.right = velocityToRPM(v_right);
    return wheel_rpm;
}

float Kinematics::rpmToVelocity(float rpm) const
{
    return rpm * 2.0f * M_PI * _wheel_radius / 60.0f;
}

float Kinematics::velocityToRPM(float velocity) const
{
    return velocity * 60.0f / (2.0f * M_PI * _wheel_radius);
}

float Kinematics::getMaxLinearVelocity(float max_rpm) const
{
    return rpmToVelocity(max_rpm);
}

float Kinematics::getMaxAngularVelocity(float max_rpm) const
{
    float max_wheel_velocity = rpmToVelocity(max_rpm);
    return (2.0f * max_wheel_velocity) / _wheel_base;
}

WheelRPM Kinematics::constrainRPM(const WheelRPM &wheel_rpm, float min_rpm, float max_rpm) const
{
    WheelRPM constrained = wheel_rpm;

    // Clamp max
    constrained.left  = std::clamp(wheel_rpm.left,  -max_rpm, max_rpm);
    constrained.right = std::clamp(wheel_rpm.right, -max_rpm, max_rpm);

    // Deadzone: nếu nhỏ hơn min thì coi như 0
    if (std::abs(constrained.left) < min_rpm) constrained.left = 0.0f;
    if (std::abs(constrained.right) < min_rpm) constrained.right = 0.0f;

    return constrained;
}

