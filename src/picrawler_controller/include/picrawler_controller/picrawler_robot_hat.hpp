#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <picrawler_interfaces/msg/onboard_button_state.hpp>


class PicrawlerRobotHat : public rclcpp::Node {
public:
    PicrawlerRobotHat();

private:
    int _pi;

    rclcpp::Publisher<picrawler_interfaces::msg::OnboardButtonState>::SharedPtr button_state_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr led_state_;

    rclcpp::TimerBase::SharedPtr button_callback_timer_;

    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void led_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void publish_button_states();
};