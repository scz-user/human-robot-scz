#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <SDL2/SDL.h>

class JoyNode : public rclcpp::Node {
public:
    JoyNode() : Node("joy_node"), joystick_(nullptr) {
        // 创建发布者
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

        // 初始化 SDL 并打开第一个手柄
        if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize SDL: %s", SDL_GetError());
            rclcpp::shutdown();
            return;
        }

        joystick_ = SDL_JoystickOpen(0);
        if (!joystick_) {
            RCLCPP_ERROR(this->get_logger(), "No joystick found.");
            SDL_Quit();
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Joystick initialized: %s", SDL_JoystickName(joystick_));

        // 创建定时器以固定频率发布手柄数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 每 100 ms 发布一次
            std::bind(&JoyNode::publishJoy, this));
    }

    ~JoyNode() {
        if (joystick_) {
            SDL_JoystickClose(joystick_);
        }
        SDL_Quit();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    SDL_Joystick* joystick_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishJoy() {
        auto joy_msg = sensor_msgs::msg::Joy();

        // 获取按键状态
        int num_buttons = SDL_JoystickNumButtons(joystick_);
        joy_msg.buttons.resize(num_buttons);
        for (int i = 0; i < num_buttons; ++i) {
            joy_msg.buttons[i] = SDL_JoystickGetButton(joystick_, i);
        }

        // 获取轴状态
        int num_axes = SDL_JoystickNumAxes(joystick_);
        joy_msg.axes.resize(num_axes);
        for (int i = 0; i < num_axes; ++i) {
            joy_msg.axes[i] = SDL_JoystickGetAxis(joystick_, i) / 32767.0;  // 归一化为 [-1, 1]
        }

        // 发布消息
        joy_pub_->publish(joy_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


