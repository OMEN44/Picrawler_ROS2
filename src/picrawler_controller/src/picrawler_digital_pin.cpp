#include <chrono>
#include <memory>

#include <picrawler_controller/picrawler_digital_pin.hpp>
#include <pigpiod_if2.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

PicrawlerDigitalPin::PicrawlerDigitalPin() 
    : Node("picrawler_digital_pin")
{
    RCLCPP_INFO(this->get_logger(), "Starting node picrawler_digital_pin");

    this->_pi = pigpio_start(NULL, NULL);
    if (this->_pi < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod");
        return;
    }

    this->_publisher = this->create_publisher<picrawler_interfaces::msg::DigitalPinState>("read_pin_state", 10);
    this->_subscription = this->create_subscription<picrawler_interfaces::msg::DigitalPinState>(
        "write_pin_state", 10, std::bind(&PicrawlerDigitalPin::set_pin_states, this, _1));
    this->_timer = this->create_wall_timer(500ms, std::bind(&PicrawlerDigitalPin::publish_pin_states, this));
}

int PicrawlerDigitalPin::read(uint8_t channel)
{
    if (channel > 3) return ERR_BAD_CHANNEL;
    set_mode(this->_pi, this->_pins[channel], PI_INPUT);
    return gpio_read(this->_pi, this->_pins[channel]);
}

int PicrawlerDigitalPin::write(uint8_t channel, bool value)
{
    if (channel > 3) return ERR_BAD_CHANNEL;
    set_mode(this->_pi, this->_pins[channel], PI_OUTPUT);
    this->_pin_states.pins[channel] = value;
    return gpio_write(this->_pi, this->_pins[channel], value);
}

void PicrawlerDigitalPin::publish_pin_states()
{
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "0: %s, 1: %s, 2: %s, 3: %s", 
    //     (this->_pin_states.pins[0] ? "true" : "false"), 
    //     (this->_pin_states.pins[1] ? "true" : "false"), 
    //     (this->_pin_states.pins[2] ? "true" : "false"), 
    //     (this->_pin_states.pins[3] ? "true" : "false"));
    // auto msg = picrawler_interfaces::msg::DigitalPinState();
    // for (int i = 0; i < 4; i++) {
    //     msg.pins[i] = this->read(i);
    // }
    // this->_pin_states = msg;
    this->_publisher->publish(this->_pin_states);
}

void PicrawlerDigitalPin::set_pin_states(const picrawler_interfaces::msg::DigitalPinState::SharedPtr targetState)
{
    RCLCPP_INFO(this->get_logger(), "Laser pin state set to: %s", (this->_pin_states.pins[3] ? "true" : "false"));
    for (int i = 0; i < 4; i++) {
        if (targetState->pins[i] != this->_pin_states.pins[i]) {
            this->write(i, targetState->pins[i]);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicrawlerDigitalPin>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}