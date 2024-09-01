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

    this->pi = pigpio_start(NULL, NULL);
    if (this->pi < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod");
        return;
    }

    // setup 12c for adc
    this->m_i2cHandle = i2c_open(this->pi, 1, this->m_i2cAddr, 0);
    if (this->m_i2cHandle < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device");
    }

    this->button_state_ = this->create_publisher<picrawler_interfaces::msg::OnboardButtonState>("onboard_button_state", 10);
    this->battery_publisher = this->create_publisher<picrawler_interfaces::msg::BatteryState>("robot_hat/battery", 10);
    // this->reset_ = this->create_subscription<std_msgs::msg::Bool>(
    //     "reset", 10, std::bind(&PicrawlerRobotHat::reset_callback, this, _1));
    this->led_state_ = this->create_subscription<std_msgs::msg::Bool>(
        "led_state", 10, std::bind(&PicrawlerRobotHat::led_state_callback, this, _1));

    this->button_callback_timer_ = this->create_wall_timer(500ms, std::bind(&PicrawlerRobotHat::publish_button_states, this));
    this->battery_callback_timer = this->create_wall_timer(100ms, std::bind(&PicrawlerRobotHat::publish_battery, this));

    // reset pin
    set_mode(this->pi, 5, PI_OUTPUT);
    reset_callback();
    // onboard led
    set_mode(this->pi, 26, PI_OUTPUT);
    // rst button
    set_mode(this->pi, 16, PI_INPUT);
    // usr button
    set_mode(this->pi, 25, PI_INPUT);

}

/// @brief I SHOULD BE A SERVICE
/// @param msg 
void PicrawlerRobotHat::reset_callback()
{
    RCLCPP_INFO(this->get_logger(), "Resetting robot hat");
    gpio_write(this->pi, 5, 0);
    time_sleep(0.01);
    gpio_write(this->pi, 5, 1);
    time_sleep(0.01);
}

void PicrawlerRobotHat::led_state_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Turning LED %s", msg->data ? "on" : "off");
    gpio_write(this->pi, 26, msg->data);
}

void PicrawlerRobotHat::publish_button_states()
{
    auto msg = picrawler_interfaces::msg::OnboardButtonState();
    msg.rst_button = !gpio_read(this->pi, 16);
    msg.usr_button = !gpio_read(this->pi, 25);
    this->button_state_->publish(msg);
}

void PicrawlerRobotHat::publish_battery()
{
    // get raw data
    int avg = 0;
    for (int i = 0; i < 10; i++) 
    {
        i2c_write_word_data(this->pi, this->m_i2cHandle, 0x17 - 4, 0);
        int result = i2c_read_byte(this->pi, this->m_i2cHandle);
        result = (result << 8) | i2c_read_byte(this->pi, this->m_i2cHandle);
        avg += result;
    }

    avg /= 10;

    // format for publishing
    auto msg = picrawler_interfaces::msg::BatteryState();
    msg.voltage = avg * 3.3 / 4095 * 3;
    msg.percent = std::round((msg.voltage - 5) / 3.4 * 100);
    // RCLCPP_INFO(this->get_logger(), "Battery voltage: %fV, %f%%", msg.voltage, (msg.voltage - 5) / 3.4 * 100);
    msg.needs_charging = (msg.percent < 20);
    // WARNING!!! Voltage below 5V will damage battery
    //Note: 8.4 is maximum voltage (Must verify this)
    this->battery_publisher->publish(msg);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicrawlerRobotHat>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}