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

WheelRPM Kinematics::constrainRPM(const WheelRPM &wheel_rpm, float max_rpm) const
{
    WheelRPM constrained = wheel_rpm;

    float max_requested = std::max(std::abs(wheel_rpm.left), std::abs(wheel_rpm.right));

    if (max_requested > max_rpm) {
        float scale = max_rpm / max_requested;
        constrained.left *= scale;
        constrained.right *= scale;
    }

    return constrained;
}
