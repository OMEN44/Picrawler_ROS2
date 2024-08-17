#include <chrono>
#include <memory>

#include "picrawler_controller/picrawler_servo_control.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

PicrawlerServoControl::PicrawlerServoControl()
    : Node("picrawler_servo_control")
{
    RCLCPP_INFO(this->get_logger(), "Starting node picrawler_servo_control");

    this->m_pi = pigpio_start(NULL, NULL);
    if (this->m_pi < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod");
        return;
    }

    this->m_i2cHandle = i2c_open(this->m_pi, 1, this->m_i2cAddr, 0);
    if (this->m_i2cHandle < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device");
        return;
    }

    resetMCU();

    for (int i = 0; i < 3; i++)
    {
        // set period to 4095
        i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x44 + i, 0xff0f);
        // set prescaler to 350 (idk y this value thats what sunfounder used)
        i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x40 + i, 0x5e01);
    }

    for (int i = 0; i < 12; i++)
    {
        i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x20 + i, 0x3301);
        time_sleep(0.1);
    }

    moveServo(0, 90);
    moveServo(0, -90);

    // setup topics
    this->legPosePublisher = this->create_publisher<picrawler_interfaces::msg::LegPose>("leg1/leg_pose", 10);
    this->legPoseTimer = this->create_wall_timer(100ms, std::bind(&PicrawlerServoControl::publishLegPose, this));
}

void PicrawlerServoControl::moveServo(uint8_t servo, float angle)
{
    if (servo > 11) return;
    RCLCPP_DEBUG(this->get_logger(), "Setting servo %d to angle %f", servo, angle);
    // determine pulse width
    angle = angle > 90 ? 90 : (angle < -90 ? -90 : angle);
    float highLevelTime = PicrawlerServoControl::map(angle, -90.f, 90.f, 500.f, 2500.f);
    int value = (int)(highLevelTime / 20000 * 4095);
    //convert to lsb
    RCLCPP_DEBUG(this->get_logger(), "Setting servo %d pulse width to %d", servo, value);
    int most = value >> 8, least = value & 0xff;
    value = (least << 8) + most;

    i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x20 + servo, value);
}

void PicrawlerServoControl::publishLegPose()
{
    picrawler_interfaces::msg::LegPose msg;
    for (int i = 0; i < 3; i++)
    {
        msg.servo_angles[i] = this->m_servoAngles[i];
    }
    this->legPosePublisher->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicrawlerServoControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    node.get()->resetMCU();
    return 0;
}