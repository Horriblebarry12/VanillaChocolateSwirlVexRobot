#include "PID.h"

#include "algorithm"

float PID::control(float value, float targetValue, float timeStep)
{
    float deltaValue = targetValue - value;
    float deltaTime = timeStep - lastTimestep;
    sum += value * deltaTime;
    float output = 0;
    output += deltaValue * P;
    if (lastValue != 0 && deltaTime != 0)
    {
        output += ((value - lastValue)/deltaTime) * I;
    }
    output += sum * D;
    return (deltaValue * P);
}

float PID::control(float value, float targetValue, float timeStep, float min, float max)
{
    return std::max(std::min(control(value, targetValue, timeStep), max), min);
} 