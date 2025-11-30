#include "vex.h"
#include <sstream>
#include <functional>
#include <string>
#include "PID.h"

#ifndef LinearTolerance
#define LinearTolerance 0.01f
#endif

#ifndef AngleTolerance
#define AngleTolerance 0.3f
#endif

#ifndef Timeout
#define Timeout 2.0f
#endif

using namespace vex;

class Drivetrain
{
public:
    float X;
    float Y;
    motor_group *LeftMotors;
    motor_group *RightMotors;

    inertial *InertialSensor;

    PID pidControllerLinear;
    PID pidControllerRotational;

    std::function<void(const std::string &)> Logger;

    Drivetrain() : X(0), Y(0)
    {
        LeftMotors = nullptr;
        RightMotors = nullptr;

        pidControllerRotational = PID(0.1f, 0.01f, 0.05f);
    }

    Drivetrain(motor_group *leftMotors, motor_group *rightMotors, inertial *inertialSensor, float kpLinear, float kiLinear, float kdLinear,
               float kpRotational, float kiRotational, float kdRotational, std::function<void(const std::string &)> logger) : X(0), Y(0)
    {
        LeftMotors = leftMotors;
        RightMotors = rightMotors;
        InertialSensor = inertialSensor;
        pidControllerLinear = PID(kpLinear, kiLinear, kdLinear);
        pidControllerRotational = PID(kpRotational, kiRotational, kdRotational);
        Logger = logger;
    }

    // Out of 100
    void SetMaxSpeed(float speedPct);
    void SetMaxTorque(float torquePct);

    void MoveToPos(float x, float y);

    // Negative values allowed
    void MoveInch(float distanceInch);
    void MoveInchPID(float distanceInch, float headingDeg = std::numeric_limits<float>::quiet_NaN());

    // Positive is counter clockwise
    void TurnToDegPID(float angleDeg);
    void TurnByDegPID(float angleDeg);
    void TurnByDeg(float angleDeg);

    void TurnWithSpeed(float speed);

private:
    float MaxSpeed = 100;
};