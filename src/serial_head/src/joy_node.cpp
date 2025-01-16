include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"

class JoyToSerialNode : public rclcpp::Node {
public:
    JoyToSerialNode() : Node("joy_to_serial_node"), serial_port_("/dev/ttyUSB0"), baud_rate_(9600), data_(0x20) {
        // 创建订阅者，订阅 /joy 话题
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToSerialNode::joyCallback, this, std::placeholders::_1));

        // 初始化串口
        try {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
            if (serial_.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            }
        } catch (const serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        }
    }

    ~JoyToSerialNode() {
        if (serial_.isOpen()) {
            serial_.close();
        }
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // 检查按键状态：LB(4号键) 和 Y(3号键)
        bool lb_pressed = msg->buttons[4];  // LB 按键
        bool y_pressed = msg->buttons[3];  // Y 按键

        if (lb_pressed && y_pressed) {
            // 按下组合键时发送数据并递增
            sendSerialData(data_);
            data_++;
            if (data_ > 0x26) {  // 限制数据范围
                data_ = 0x20;
            }
        }
    }

    void sendSerialData(uint8_t value) {
        if (serial_.isOpen()) {
            uint8_t data_to_send = value;
            serial_.write(&data_to_send, 1);  // 写入单字节数据
            RCLCPP_INFO(this->get_logger(), "Sent: 0x%02X", value);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open. Data not sent.");
        }
    }

    // ROS 2 节点成员
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    // 串口相关
    serial::Serial serial_;
    std::string serial_port_;
    uint32_t baud_rate_;
    uint8_t data_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyToSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}