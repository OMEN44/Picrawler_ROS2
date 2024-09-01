
#define PRESCALER_REG 0x40
#define CHANNEL_REG 0x20
#define PERIOD_REG 0x44
#define CLOCK 72000000

class Servo
{
private:
    float angle;
    unsigned channel;
    float pulseWidth;
    float frequency;
    int i2chandle;
    int piHandle;

    void setPrescaler(int prescaler);
    void setPeriod(int period);

    static int map(int value, int valueMax, int valueMin, int outputMax, int outputMin)
    {
        return (value - valueMin) * (outputMax - outputMin) / (valueMax - valueMin) + outputMin;
    }

    static float map(float value, float valueMax, float valueMin, float outputMax, float outputMin)
    {
        return (value - valueMin) * (outputMax - outputMin) / (valueMax - valueMin) + outputMin;
    }

public:
    Servo(unsigned channel, int pihandle, int i2cHandle);

    int setAngle(float angle);
    float getAngle();
};
