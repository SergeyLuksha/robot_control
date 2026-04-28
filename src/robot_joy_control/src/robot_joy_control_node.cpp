#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoystickTeleopNode : public rclcpp::Node {
public:
    JoystickTeleopNode() : Node("joystick_teleop_node") {
        // Параметры для настройки чувствительности
        this->declare_parameter("linear_scale", 0.5); // м/с
        this->declare_parameter("angular_scale", 1.0); // рад/с

        // Индексы осей (для Logitech F710: левый стик вверх/вниз и левый стик влево/вправо)
        // В режиме XInput: ось 1 - левый стик верт., ось 0 - левый стик гор.
        this->declare_parameter("axis_linear", 1);
        this->declare_parameter("axis_angular", 3); // часто правый стик или 0 для левого

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickTeleopNode::joy_callback, this, std::placeholders::_1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Joystick Teleop Node started.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto twist = geometry_msgs::msg::Twist();

        double linear_scale = this->get_parameter("linear_scale").as_double();
        double angular_scale = this->get_parameter("angular_scale").as_double();
        int linear_axis = this->get_parameter("axis_linear").as_int();
        int angular_axis = this->get_parameter("axis_angular").as_int();

        // Проверка на наличие осей в сообщении во избежание segfault
        if (msg->axes.size() > (size_t)std::max(linear_axis, angular_axis)) {
            twist.linear.x = msg->axes[linear_axis] * linear_scale;
            twist.angular.z = msg->axes[angular_axis] * angular_scale;
        }

        twist_pub_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickTeleopNode>());
    rclcpp::shutdown();
    return 0;
}
