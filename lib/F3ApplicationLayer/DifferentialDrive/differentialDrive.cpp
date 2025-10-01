#include "differentialDrive.h"
#include <Arduino.h>

DifferentialDrive::DifferentialDrive(Wheel *wheel_left, Wheel *wheel_right, float wheel_radius, float wheel_base, float min_rpm_percent, float max_rpm_percent)
    : _wheel_left(wheel_left), _wheel_right(wheel_right), _kinematics(wheel_radius, wheel_base), _min_rpm_percent(min_rpm_percent), _max_rpm_percent(max_rpm_percent), _printCounter(0)
{
    _target_velocity = {0.0f, 0.0f};
    _current_velocity = {0.0f, 0.0f};
}

void DifferentialDrive::setVelocity(float linear, float angular)
{
    RobotVelocity velocity = {linear, angular};
    setVelocity(velocity);
}

void DifferentialDrive::setVelocity(const RobotVelocity &velocity)
{
    _target_velocity = velocity;

    WheelRPM target_rpm = _kinematics.inverseKinematics(velocity);
    target_rpm = _kinematics.constrainRPM(target_rpm, _min_rpm_percent, _max_rpm_percent);

    _wheel_left->setTargetRPM(target_rpm.left);
    _wheel_right->setTargetRPM(target_rpm.right);
}

void DifferentialDrive::moveLinear(float speed)
{
    setVelocity(speed, 0.0f);
}

void DifferentialDrive::moveForward(float speed)
{
    setVelocity(speed, 0.0f);
}

void DifferentialDrive::moveBackward(float speed)
{
    setVelocity(-speed, 0.0f);
}

void DifferentialDrive::rotate(float angular_speed)
{
    setVelocity(0.0f, angular_speed);
}

void DifferentialDrive::turnLeft(float angular_speed)
{
    setVelocity(0.0f, angular_speed); // positive value, ccw
}

void DifferentialDrive::turnRight(float angular_speed)
{
    setVelocity(0.0f, -angular_speed); // negative value, cw
}

void DifferentialDrive::stop()
{
    setVelocity(0.0f, 0.0f);
}

void DifferentialDrive::update()
{
    WheelRPM current_rpm;
    current_rpm.left = _wheel_left->getCurrentRPM();
    current_rpm.right = _wheel_right->getCurrentRPM();
    _current_velocity = _kinematics.forwardKinematics(current_rpm);
}

RobotVelocity DifferentialDrive::getCurrentVelocity() const
{
    return _current_velocity;
}

WheelRPM DifferentialDrive::getCurrentWheelRPM() const
{
    WheelRPM current_rpm;
    current_rpm.left = _wheel_left->getCurrentRPM();
    current_rpm.right = _wheel_right->getCurrentRPM();
    return current_rpm;
}

WheelRPM DifferentialDrive::getTargetWheelRPM() const
{
    return _kinematics.inverseKinematics(_target_velocity);
}

void DifferentialDrive::printStatus(bool active, uint8_t print_interval) const
{
    if (!active)
        return;
    _printCounter++;
    if (_printCounter == print_interval)
    {
        Serial.print("SP_Linear:");
        Serial.print(_target_velocity.linear);
        Serial.print(",");
        Serial.print("SP_Angular:");
        Serial.print(_target_velocity.angular);
        Serial.print(",");
        Serial.print("C_Linear:");
        Serial.print(_current_velocity.linear);
        Serial.print(",");
        Serial.print("C_Angular:");
        Serial.print(_current_velocity.angular);
        Serial.println();
        _printCounter = 0;
    }
}
