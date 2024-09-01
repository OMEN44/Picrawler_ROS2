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

    // init the servos
    for (int i = 0; i < 3; i++)
    {
        // set period to 4095
        i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x44 + i, 0xff0f);
        // set prescaler to 350 (idk y this value thats what sunfounder used)
        i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x40 + i, 0x5e01);
    }

    // zero all servos
    for (unsigned i = 0; i < 12; i++)
    {
        i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x20 + i, 0x3301);
        time_sleep(0.1);
    }

    // setup topics
    this->legPosePublisher = this->create_publisher<picrawler_interfaces::msg::LegPose>("leg1/leg_pose", 10);
    this->legPoseTimer = this->create_wall_timer(100ms, std::bind(&PicrawlerServoControl::publishLegPose, this));

    setServoAngle(1, 45);

    // this->controlSubscriber = this->create_subscription<std_msgs::msg::Int8MultiArray>(
    //     "teleop/keyboard", 2, std::bind(&PicrawlerServoControl::controlCallback, this, _1));
}

void PicrawlerServoControl::setServoAngle(uint8_t servo, float angle)
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
    // time_sleep(0.01);
    this->m_servoAngles[servo] = angle;
}

void PicrawlerServoControl::moveLegs(float targets[4][3])
{
    
}

// void PicrawlerServoControl::moveLeg(uint8_t leg, float[3] angles)
// {

//     if (leg > 3) return;

//     float c = std::sqrt((x * x) + (y * y));
//     if (c > 125) return;
//     float a1 = std::acos(-((c * c) - 8125) / 7500) * 180 / M_PI - 90;
//     // clamp between 0 and 90
//     float a2 = 90 - (std::acos((5625 - (2500 + c * c))/(-100 * c)) * 180 / M_PI) - (atan(y / x) * 180 / M_PI);
//     a2 = a2 < 0 ? 0 : (a2 > 90 ? 90 : a2);

//     setServoAngle(3 * leg, a1);
//     setServoAngle(3 * leg + 1, a2);
//     setServoAngle(3 * leg + 2, angles[2] > 45 ? 45 : (angles[2] < -45 ? -45 : angles[2]));

//     RCLCPP_INFO(
//         this->get_logger(), 
//         "\nx: %f, y: %f, c: %f \ns0: %f, s1: %f",
//         x,
//         y,
//         c,
//         this->m_servoAngles[0],
//         this->m_servoAngles[1]
//     );
// }

uint16_t PicrawlerServoControl::analogRead(uint8_t channel)
{
    if (channel > 3) return -1;
    i2c_write_word_data(this->m_pi, this->m_i2cHandle, 0x17 - channel, 0);
    int result = i2c_read_byte(this->m_pi, this->m_i2cHandle);
    result = (result << 8) | i2c_read_byte(this->m_pi, this->m_i2cHandle);
    return result;
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

void PicrawlerServoControl::controlCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 4) return;



    if (msg->data[0] != 0 || msg->data[1] != 0 || msg->data[2] != 0 || msg->data[3] != 0)
    {
        this->x += msg->data[0] + msg->data[1];
        this->y += msg->data[2] + msg->data[3];
        
        float x = this->x < 0 ? 0 : (this->x > 125 ? 125 : this->x);
        float y = this->y < -125 ? -125 : (this->y > 125 ? 125 : this->y);

        // moveLeg(0, x, y, 20);
        // s1.setAngle(x);
    }
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