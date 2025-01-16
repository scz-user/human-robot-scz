#ifndef SERIAL_SENDER_H
#define SERIAL_SENDER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <string>

class SerialSender {
public:
    /**
     * @brief 构造函数，初始化串口和订阅者
     * @param port 串口端口号（如 /dev/ttyUSB0）
     * @param baud_rate 串口波特率
     */
    SerialSender(const std::string& port, uint32_t baud_rate);

    /**
     * @brief 析构函数，关闭串口
     */
    ~SerialSender();

private:
    ros::NodeHandle nh;               // ROS 节点句柄
    ros::Subscriber joy_sub;          // 手柄订阅者
    serial::Serial ser;               // 串口对象
    uint8_t eye_mode;                 // 当前模式值
    bool last_button_state;           // 上次按键状态

    /**
     * @brief 手柄按键回调函数
     * @param msg 手柄消息
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    /**
     * @brief 通过串口发送单字节数据
     * @param value 要发送的数据
     */
    void sendData(uint8_t value);
};

#endif // SERIAL_SENDER_H