#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include "../Kinematics/kinematics.h"
#include "../../F2ControlLayer/Wheel/wheel.h"

class DifferentialDrive {
private:
    Wheel * _wheel_left;
    Wheel * _wheel_right;  
    Kinematics _kinematics;

    RobotVelocity _target_velocity;
    RobotVelocity _current_velocity;

    float _max_rpm_percent;
    float _min_rpm_percent;
    mutable uint8_t _printCounter;

public:
    DifferentialDrive(Wheel * wheel_left, Wheel * wheel_right, 
        float wheel_radius = 0.017f, float wheel_base = 0.075f, 
        float min_rpm_percent = 20.0f, float max_rpm_percent = 60.0f);

    void setVelocity(float linear, float angular);
    void setVelocity(const RobotVelocity &velocity);

    void moveLinear(float speed); // positive for forward, negative for backward
    void moveForward(float speed); // positive value, move forward
    void moveBackward(float speed); // positive value, move backward
    void rotate(float angular_speed);
    void turnLeft(float angular_speed);
    void turnRight(float angular_speed);

    void stop();
    void update();
    
    RobotVelocity getCurrentVelocity() const;
    RobotVelocity getTargetVelocity() const { return _target_velocity; }

    WheelRPM getCurrentWheelRPM() const;
    WheelRPM getTargetWheelRPM() const;

    void setMaxRPM(float max_rpm_percent) { _max_rpm_percent = max_rpm_percent; }
    void setMinRPM(float min_rpm_percent) { _min_rpm_percent = min_rpm_percent; }
    float getMaxRPM() const { return _max_rpm_percent; }
    float getMinRPM() const { return _min_rpm_percent; }

    void printStatus(bool active, uint8_t print_interval) const;

    Kinematics& getKinematics() { return _kinematics; }
    
    Wheel* getWheelLeft() { return _wheel_left; }
    Wheel* getWheelRight() { return _wheel_right; }
};

#endif