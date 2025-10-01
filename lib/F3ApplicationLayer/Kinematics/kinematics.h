#ifndef KINEMATICS_H
#define KINEMATICS_H

struct RobotVelocity {
    float linear;   //m/s 
    float angular;  //rad/s
};

struct WheelRPM {
    float left;
    float right;
};

class Kinematics {
private:
    float _wheel_radius;
    float _wheel_base;

public:
    Kinematics(float wheel_radius = 0.017f, float wheel_base = 0.075f);
    RobotVelocity forwardKinematics(const WheelRPM& wheel_rpm) const;
    WheelRPM inverseKinematics(const RobotVelocity& robot_vel) const;

    float rpmToVelocity(float rpm) const;
    float velocityToRPM(float velocity) const;
    
    float getWheelRadius() const { return _wheel_radius; }
    float getWheelBase() const { return _wheel_base; }

    void setWheelRadius(float wheel_radius) { _wheel_radius = wheel_radius; }
    void setWheelBase(float wheel_base) { _wheel_base = wheel_base; }

    float getMaxLinearVelocity(float max_rpm) const;
    float getMaxAngularVelocity(float max_rpm) const;

    WheelRPM constrainRPM(const WheelRPM& wheel_rpm, float min_rpm, float max_rpm) const;
};

#endif