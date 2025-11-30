#include "Drivetrain.h"
#include "cmath"
const float wheelCircumferenceInch = 3.25 * 3.14159;
const float gearRatio = 60.0f / 36.0f;
const float wheelBaseInch = 12.0f;

float angleBetween(float alpha, float beta)
{
    float delta = fmodf(beta - alpha + 180.0f, 360.0f) - 180.0f;
    return delta < -180.0f ? delta + 360.0f : delta;
}

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

    MaxSpeed = speedPct;
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

void Drivetrain::MoveToPos(float x, float y)
{
    float deltaX = x - X;
    float deltaY = y - Y;
    float distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    float targetHeading = std::atan2(deltaY, deltaX) * (180.0f / 3.14159f);

    MoveInchPID(distance, targetHeading);
}

void Drivetrain::MoveInchPID(float distanceInch, float headingDeg)
{
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        float rotations = (distanceInch / wheelCircumferenceInch) * gearRatio;

        float timer = 0;
        bool isTiming = false;
        float targetHeading = InertialSensor->heading();
        if (!std::isnan(headingDeg))
        {
            targetHeading = headingDeg;
            TurnToDegPID(headingDeg);
        }

        X += std::cos(InertialSensor->heading() * (3.14159f / 180.0f)) * distanceInch;
        Y += std::sin(InertialSensor->heading() * (3.14159f / 180.0f)) * distanceInch;

        pidControllerLinear.reset();
        pidControllerRotational.reset();

        float currentHeading = InertialSensor->heading();

        float offsetLeft = LeftMotors->position(rotationUnits::rev);
        float offsetRight = RightMotors->position(rotationUnits::rev);

        float posLeft = LeftMotors->position(rotationUnits::rev) - offsetLeft;
        float posRight = RightMotors->position(rotationUnits::rev) - offsetRight;
        float avgPos = (posLeft - posRight) / 2.0f;
        float lastTimestamp = InertialSensor->timestamp() / 1000.0f;
        float deltaTime = 0;
        float speed = 0;
        float accelerationTime = 100.0f;
        float timeSinceStart = 0.0f;
        while (fabs(avgPos - rotations) > LinearTolerance && timer < Timeout)
        {
            deltaTime = (InertialSensor->timestamp() - lastTimestamp) / 1000.0f;
            if (deltaTime == 0)
                continue;
            if (isTiming)
                timer += deltaTime;
            timeSinceStart += deltaTime;
            timeSinceStart = std::min(timeSinceStart, accelerationTime);
            posLeft = LeftMotors->position(rotationUnits::rev) - offsetLeft;
            posRight = RightMotors->position(rotationUnits::rev) - offsetRight;
            avgPos = (posLeft - posRight) / 2.0f;
            currentHeading = InertialSensor->heading();
            speed = (speed * 0.5f) + (0.5f * pidControllerLinear.control((avgPos/gearRatio) * wheelCircumferenceInch, (rotations/gearRatio) * wheelCircumferenceInch, deltaTime, (-20 * 0.9f) * (timeSinceStart / accelerationTime), (20 * 0.9f) * (timeSinceStart / accelerationTime)));
            float headingCorrection = pidControllerRotational.control(angleBetween(currentHeading, targetHeading), deltaTime, -60, 60);
            LeftMotors->spin(directionType::fwd, -speed + headingCorrection, velocityUnits::pct);
            RightMotors->spin(directionType::fwd, speed + headingCorrection, velocityUnits::pct);

            if (fabs(avgPos - rotations) < LinearTolerance)
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
        }
        LeftMotors->stop();
        RightMotors->stop();
    }
}

void Drivetrain::MoveInch(float distanceInch)
{
    float rotations = (distanceInch / wheelCircumferenceInch) * gearRatio;
    X += std::cos(InertialSensor->heading() * (3.14159f / 180.0f)) * distanceInch;
    Y += std::sin(InertialSensor->heading() * (3.14159f / 180.0f)) * distanceInch;
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        LeftMotors->spinFor(-rotations, rotationUnits::rev, false);
        RightMotors->spinFor(rotations, rotationUnits::rev, true);
    }
}

void Drivetrain::TurnToDegPID(float angleDeg)
{
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        float timer = 0;
        bool isTiming = false;

        pidControllerRotational.reset();
        float targetAngle = angleDeg;

        float currentAngle = InertialSensor->heading();
        float lastTimestamp = InertialSensor->timestamp() / 1000.0f;
        float deltaTime = 0;

        while (fabs(angleBetween(currentAngle, targetAngle)) > AngleTolerance && timer < Timeout)
        {
            deltaTime = (InertialSensor->timestamp() - lastTimestamp) / 1000.0f;
            if (deltaTime == 0)
                continue;
            if (isTiming)
                timer += deltaTime;
            currentAngle = InertialSensor->heading();
            TurnWithSpeed(pidControllerRotational.control(-angleBetween(currentAngle, targetAngle), deltaTime, -MaxSpeed, MaxSpeed));

            if (fabs(angleBetween(currentAngle, targetAngle)) < AngleTolerance)
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
        }

        LeftMotors->stop();
        RightMotors->stop();
    }
}
void Drivetrain::TurnByDegPID(float angleDeg)
{
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        float timer = 0;
        bool isTiming = false;

        pidControllerRotational.reset();
        float targetAngle = InertialSensor->heading() + angleDeg;

        float currentAngle = InertialSensor->heading();
        float lastTimestamp = InertialSensor->timestamp() / 1000.0f;
        float deltaTime = 0;

        while (fabs(angleBetween(currentAngle, targetAngle)) > AngleTolerance && timer < Timeout)
        {
            deltaTime = (InertialSensor->timestamp() - lastTimestamp) / 1000.0f;
            if (deltaTime == 0)
                continue;
            if (isTiming)
                timer += deltaTime;
            currentAngle = InertialSensor->heading();
            TurnWithSpeed(pidControllerRotational.control(-angleBetween(currentAngle, targetAngle), deltaTime, -MaxSpeed, MaxSpeed));

            if (fabs(angleBetween(currentAngle, targetAngle)) < AngleTolerance)
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
        }

        LeftMotors->stop();
        RightMotors->stop();
    }
}

void Drivetrain::TurnByDeg(float angleDeg)
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

void Drivetrain::TurnWithSpeed(float speed)
{
    if (LeftMotors != nullptr && RightMotors != nullptr)
    {
        LeftMotors->spin(directionType::fwd, -speed, velocityUnits::pct);
        RightMotors->spin(directionType::fwd, -speed, velocityUnits::pct);
    }
}