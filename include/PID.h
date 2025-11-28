class PID
{
public:
    float P;
    float I;
    float D;

    float sum = 0;

    float lastTimestep = 0;
    float lastValue = 0;

    PID() : P(0), I(0), D(0)
    {
        sum = 0;
        lastTimestep = 0;
        lastValue = 0;
    }

    PID(float p, float i, float d) : P(p), I(i), D(d)
    {
        sum = 0;
        lastTimestep = 0;
        lastValue = 0;
    }

    
    float control(float value, float targetValue, float timeStep);
    float control(float value, float targetValue, float timeStep, float min, float max);

    void reset();
private:

};