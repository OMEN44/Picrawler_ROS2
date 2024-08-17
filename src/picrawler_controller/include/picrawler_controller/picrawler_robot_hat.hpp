#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <picrawler_interfaces/msg/onboard_button_state.hpp>
#include <picrawler_interfaces/msg/battery_state.hpp>

class PicrawlerRobotHat : public rclcpp::Node {
public:
    PicrawlerRobotHat();

private:
    int pi;
    int m_i2cHandle;
    unsigned m_i2cAddr = 0x14; // 20

    rclcpp::Publisher<picrawler_interfaces::msg::OnboardButtonState>::SharedPtr button_state_;
    rclcpp::Publisher<picrawler_interfaces::msg::BatteryState>::SharedPtr battery_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr led_state_;

    rclcpp::TimerBase::SharedPtr button_callback_timer_;
    rclcpp::TimerBase::SharedPtr battery_callback_timer;

    void reset_callback();
    void led_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void publish_button_states();
    void publish_battery();
};