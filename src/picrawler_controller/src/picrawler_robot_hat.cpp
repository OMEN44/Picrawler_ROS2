#include <memory>
#include <chrono>

#include "picrawler_controller/picrawler_robot_hat.hpp"
#include <pigpiod_if2.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

PicrawlerRobotHat::PicrawlerRobotHat() 
    : Node("picrawler_robot_hat")
{
    RCLCPP_INFO(this->get_logger(), "Starting node picrawler_robot_hat");

    this->_pi = pigpio_start(NULL, NULL);
    if (this->_pi < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod");
        return;
    }

    this->button_state_ = this->create_publisher<picrawler_interfaces::msg::OnboardButtonState>("onboard_button_state", 10);
    this->reset_ = this->create_subscription<std_msgs::msg::Bool>(
        "reset", 10, std::bind(&PicrawlerRobotHat::reset_callback, this, _1));
    this->led_state_ = this->create_subscription<std_msgs::msg::Bool>(
        "led_state", 10, std::bind(&PicrawlerRobotHat::led_state_callback, this, _1));

    this->button_callback_timer_ = this->create_wall_timer(500ms, std::bind(&PicrawlerRobotHat::publish_button_states, this));

    // reset pin
    set_mode(this->_pi, 5, PI_OUTPUT);
    // onboard led
    set_mode(this->_pi, 26, PI_OUTPUT);
    // rst button
    set_mode(this->_pi, 16, PI_INPUT);
    // usr button
    set_mode(this->_pi, 25, PI_INPUT);
}

/// @brief I SHOULD BE A SERVICE
/// @param msg 
void PicrawlerRobotHat::reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Resetting robot hat");
        gpio_write(this->_pi, 5, 0);
        time_sleep(0.01);
        gpio_write(this->_pi, 5, 1);
        time_sleep(0.01);
    }
}

void PicrawlerRobotHat::led_state_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Turning LED %s", msg->data ? "on" : "off");
    gpio_write(this->_pi, 26, msg->data);
}

void PicrawlerRobotHat::publish_button_states()
{
    auto msg = picrawler_interfaces::msg::OnboardButtonState();
    msg.rst_button = !gpio_read(this->_pi, 16);
    msg.usr_button = !gpio_read(this->_pi, 25);
    this->button_state_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicrawlerRobotHat>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}