#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pigpiod_if2.h>
#include "picrawler_interfaces/msg/leg_pose.hpp"

class PicrawlerServoControl : public rclcpp::Node
{
public:
    PicrawlerServoControl();

    void resetMCU()
    {
        set_mode(this->m_pi, 5, PI_OUTPUT);
        gpio_write(this->m_pi, 5, 0);
        time_sleep(0.01);
        gpio_write(this->m_pi, 5, 1);
        time_sleep(0.01);
    }

private:
    int m_pi;
    int m_i2cHandle;
    unsigned m_i2cAddr = 0x14; // 20
    float m_servoAngles[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rclcpp::TimerBase::SharedPtr legPoseTimer;

    rclcpp::Publisher<picrawler_interfaces::msg::LegPose>::SharedPtr legPosePublisher;

    void moveServo(uint8_t servo, float angle);
    void moveLeg(uint8_t leg, float x, float y, float z);
    void publishLegPose();

    static int map(int value, int valueMax, int valueMin, int outputMax, int outputMin)
    {
        return (value - valueMin) * (outputMax - outputMin) / (valueMax - valueMin) + outputMin;
    }

    static float map(float value, float valueMax, float valueMin, float outputMax, float outputMin)
    {
        return (value - valueMin) * (outputMax - outputMin) / (valueMax - valueMin) + outputMin;
    }
};
