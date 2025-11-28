#include "Drivetrain.h"
#include "cmath"
const float wheelCircumferenceInch = 3.25 * 3.14159;
const float gearRatio = 60.0f / 36.0f;
const float wheelBaseInch = 12.0f;

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
    float rotations = (distanceInch / wheelCircumferenceInch) * gearRatio;
    X += std::cos(InertialSensor->heading()) * distanceInch;
    Y += std::sin(InertialSensor->heading()) * distanceInch;
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        LeftMotors->spinFor(-rotations, rotationUnits::rev, false);
        RightMotors->spinFor(rotations, rotationUnits::rev, true);
    }
}

void Drivetrain::MoveCommandTile(float distanceTile)
{
    MoveCommandInch(distanceTile * 12.0f); // Assuming 1 tile = 12 inches
}

void Drivetrain::MoveCommandMM(float distanceMM)
{
    MoveCommandInch(distanceMM / 25.4f);
}

void Drivetrain::TurnCommandDegPID(float angleDeg)
{
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        float timer = 0;
        bool isTiming = false;

        pidController.reset();
        float targetAngle = InertialSensor->heading() + angleDeg;

        float currentAngle = InertialSensor->heading();
        float lastTimestamp = InertialSensor->timestamp() / 1000.0f;
        float deltaTime = 0;

        while (fabs(currentAngle - targetAngle) > 0.2f && timer < 3)
        {
            deltaTime = (InertialSensor->timestamp() - lastTimestamp) / 1000.0f;
            if (deltaTime == 0)
                continue;
            if (isTiming)
                timer += deltaTime;
            currentAngle = InertialSensor->heading();
            TurnCommandSpeed(pidController.control(currentAngle, targetAngle, deltaTime, -100.0f, 100.0f));

            if (fabs(currentAngle - targetAngle) < 0.2f)
            {
                if (!isTiming)
                    isTiming = true;
            }
            else
            {
                isTiming = false;
                timer = 0;
            }

            lastTimestamp = InertialSensor->timestamp() / 1000.0f;
            // wait(50, timeUnits::msec);
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
    float rotations = (distanceInch / wheelCircumferenceInch) * gearRatio;

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
        LeftMotors->spin(directionType::fwd, -speed, velocityUnits::pct);
        RightMotors->spin(directionType::fwd, -speed, velocityUnits::pct);
    }
}