#include "vex.h"
#include <sstream>
#include <functional>
#include <string>
#include "PID.h"

using namespace vex;

class Drivetrain
{
public:
    float X;
    float Y;
    motor_group *LeftMotors;
    motor_group *RightMotors;

    inertial *InertialSensor;

    PID pidController;

    std::function<void(const std::string &)> Logger;

    Drivetrain() : X(0), Y(0)
    {
        LeftMotors = nullptr;
        RightMotors = nullptr;

        pidController = PID(0.1f, 0.01f, 0.05f);
    }

    Drivetrain(motor_group *leftMotors, motor_group *rightMotors, inertial *inertialSensor, float kp, float ki, float kd, 
        std::function<void(const std::string &)> logger) : X(0), Y(0)
    {
        LeftMotors = leftMotors;
        RightMotors = rightMotors;
        InertialSensor = inertialSensor;
        pidController = PID(kp, ki, kd);
        Logger = logger;
    }

    // Out of 100
    void SetMaxSpeed(float speedPct);
    void SetMaxTorque(float torquePct);

    // Negative values allowed
    void MoveCommandInch(float distanceInch);
    void MoveCommandTile(float distanceTile);
    void MoveCommandMM(float distanceMM);

    // Positive is counter clockwise
    void TurnCommandDegPID(float angleDeg);
    void TurnCommandRadPID(float angleRad);

    void TurnCommandDeg(float angleDeg);
    void TurnCommandRad(float angleRad);

    void TurnCommandSpeed(float speed);

private:
};