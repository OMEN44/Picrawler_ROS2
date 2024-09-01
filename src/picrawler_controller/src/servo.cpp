#include <pigpiod_if2.h>
#include <cmath>
#include <vector>

#include "picrawler_controller/servo.hpp"

Servo::Servo(unsigned channel, int piHandle, int i2cHandle) 
{
    this->channel = channel;
    this->angle = 0.0;
    this->pulseWidth = 0.0;
    this->piHandle = piHandle;
    this->i2chandle = i2cHandle;

    // Set the frequency to 50 Hz
    this->frequency = 50.0;
    // int st = std::sqrt(CLOCK / this->frequency);

    // st -= 5;

    // if (st < 0) st = 1;

    // float bestAccuracy = 10000000;
    // int bestPsc;
    // int bestArr;

    // for (int psc = st; psc < st + 10; psc++)
    // {
    //     int arr = CLOCK / this->frequency / psc;

    //     float accuracy = std::abs(this->frequency - CLOCK / psc / arr);

    //     if (accuracy < bestAccuracy) 
    //     {
    //         bestPsc = psc;
    //         bestArr = arr;
    //     }
    // }

    this->setPeriod(4095);
    this->setPrescaler(CLOCK / this->frequency / 4095);
}

void Servo::setPrescaler(int prescaler) 
{
    prescaler -= 1;
    int reg = PRESCALER_REG + this->channel / 4;
    int wordData = (prescaler & 0xff00) + ((prescaler & 0xff) << 8);
    i2c_write_word_data(this->piHandle, this->i2chandle, reg, wordData);

}

void Servo::setPeriod(int period)
{
    period -= 1;
    int reg = PERIOD_REG + this->channel / 4;
    int wordData = (period & 0xff00) + ((period & 0xff) << 8);
    i2c_write_word_data(this->piHandle, this->i2chandle, reg, wordData);
}

int Servo::setAngle(float angle)
{

    // determine pulse width
    angle = angle > 90 ? 90 : (angle < -90 ? -90 : angle);
    float highLevelTime = Servo::map(angle, -90.f, 90.f, 500.f, 2500.f);
    int value = (int)(highLevelTime / 20000 * 4095);
    //convert to lsb
    int most = value >> 8, least = value & 0xff;
    value = (least << 8) + most;

    // i2c_write_word_data(this->piHandle, this->i2chandle, CHANNEL_REG + this->channel, value);
    // time_sleep(0.01);
    this->angle = angle;
    return value;
}