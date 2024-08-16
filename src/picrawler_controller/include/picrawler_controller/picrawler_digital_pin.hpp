#pragma once

#include "rclcpp/rclcpp.hpp"
#include "picrawler_interfaces/msg/digital_pin_state.hpp"

#define HIGH 1
#define LOW 0

#define ERR_BAD_CHANNEL -2

class PicrawlerDigitalPin : public rclcpp::Node {
public:
    PicrawlerDigitalPin();

private:
    int _pi;
    int _i2c_handle;
    const uint8_t _pins[4] = {17, 4, 27, 22};
    picrawler_interfaces::msg::DigitalPinState _pin_states = picrawler_interfaces::msg::DigitalPinState();

    rclcpp::Publisher<picrawler_interfaces::msg::DigitalPinState>::SharedPtr _publisher;
    rclcpp::Subscription<picrawler_interfaces::msg::DigitalPinState>::SharedPtr _sub_write;
    rclcpp::Subscription<picrawler_interfaces::msg::DigitalPinState>::SharedPtr _sub_set;
    rclcpp::TimerBase::SharedPtr _timer;

    int read(uint8_t channel);
    int write(uint8_t channel, bool value);

    void publish_pin_states();
    void write_to_pin(const picrawler_interfaces::msg::DigitalPinState::SharedPtr targetState);
    void set_pin_states(const picrawler_interfaces::msg::DigitalPinState::SharedPtr targetState);

};