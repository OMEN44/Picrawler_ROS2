#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pigpiod_if2.h>
#include "picrawler_interfaces/msg/leg_pose.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "picrawler_controller/servo.hpp"

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
    float m_legPositions[4][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    int x = 0, y = 0;

    rclcpp::TimerBase::SharedPtr legPoseTimer;

    rclcpp::Publisher<picrawler_interfaces::msg::LegPose>::SharedPtr legPosePublisher;
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr controlSubscriber;

    void setServoAngle(uint8_t servo, float angle);
    void moveLeg(uint8_t leg, float angles[3]);
    void moveLegs(float targets[4][3]);
    void publishLegPose();
    void controlCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);

    // for testing
    uint16_t analogRead(uint8_t channel);

    static int map(int value, int valueMax, int valueMin, int outputMax, int outputMin)
    {
        return (value - valueMin) * (outputMax - outputMin) / (valueMax - valueMin) + outputMin;
    }

    static float map(float value, float valueMax, float valueMin, float outputMax, float outputMin)
    {
        return (value - valueMin) * (outputMax - outputMin) / (valueMax - valueMin) + outputMin;
    }
};
