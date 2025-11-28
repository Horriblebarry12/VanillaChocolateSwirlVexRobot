#include "Drivetrain.h"
#include "vex.h"

const float wheelCircumferenceInch = 4 * 3.14159; // Assuming 4 inch diameter wheels
const float wheelBaseInch = 12.0f; // Distance between left and right wheels


void Drivetrain::SetMaxSpeed(float speedPct)
{
    if (LeftMotors != nullptr)
    {
        LeftMotors->setVelocity(speedPct, percentUnits::pct);
    }
    if (RightMotors != nullptr)
    {
        RightMotors->setVelocity(speedPct, percentUnits::pct);
    }
}

void Drivetrain::SetMaxTorque(float torquePct)
{
    if (LeftMotors != nullptr)
    {
        LeftMotors->setMaxTorque(torquePct, percentUnits::pct);
    }
    if (RightMotors != nullptr)
    {
        RightMotors->setMaxTorque(torquePct, percentUnits::pct);
    }
}

void Drivetrain::MoveCommandInch(float distanceInch)
{
    float rotations = distanceInch / wheelCircumferenceInch;

    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        LeftMotors->spinFor(rotations, rotationUnits::rev, false);
        RightMotors->spinFor(rotations, rotationUnits::rev, true);
    }
}

void Drivetrain::MoveCommandTile(float distanceTile)
{
    MoveCommandInch(distanceTile * 12.0f); // Assuming 1 tile = 12 inches
}

void Drivetrain::MoveCommandMM(float distanceMM)
{
    MoveCommandInch(distanceMM / 25.4f); // Convert mm to inches
}

void Drivetrain::TurnCommandDegPID(float angleDeg)
{
    float turnCircumferenceInch = wheelBaseInch * 3.14159f;
    float distanceInch = (angleDeg / 360.0f) * turnCircumferenceInch;
    float rotations = distanceInch / wheelCircumferenceInch;

    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        float targetAngle = InertialSensor->heading() + angleDeg;

        float currentAngle = InertialSensor->heading();
        float lastTimestamp = InertialSensor->timestamp() / 1000.0f;
        float deltaTime = 0;
        while (fabs(currentAngle - targetAngle) > 1.0f)
        {
            deltaTime = (InertialSensor->timestamp() - lastTimestamp) / 1000.0f; 
            if (deltaTime == 0)
                continue;
            currentAngle = InertialSensor->heading();
            TurnCommandSpeed(pidController.control(currentAngle, targetAngle, deltaTime, -100.0f, 100.0f));

            lastTimestamp = InertialSensor->timestamp() / 1000.0f;

        }

        LeftMotors->stop();
        RightMotors->stop();
    }
}

void Drivetrain::TurnCommandRadPID(float angleRad)
{
    TurnCommandDegPID(angleRad * (180.0f / 3.14159f));
}

void Drivetrain::TurnCommandDeg(float angleDeg)
{
    float turnCircumferenceInch = wheelBaseInch * 3.14159f;
    float distanceInch = (angleDeg / 360.0f) * turnCircumferenceInch;
    float rotations = distanceInch / wheelCircumferenceInch;

    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        LeftMotors->spinFor(rotations, rotationUnits::rev, false);
        RightMotors->spinFor(-rotations, rotationUnits::rev, true);
    }
    
}

void Drivetrain::TurnCommandRad(float angleRad)
{
    TurnCommandDeg(angleRad * (180.0f / 3.14159f));
}

void Drivetrain::TurnCommandSpeed(float speed)
{
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        LeftMotors->spin(directionType::fwd, speed, velocityUnits::pct);
        RightMotors->spin(directionType::rev, speed, velocityUnits::pct);
    }
}