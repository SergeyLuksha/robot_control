#include <memory>
#include <string>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using boost::asio::ip::udp;

// Выравнивание структур должно быть идентично ESP32
#pragma pack(push, 1)
struct CommandPacket {
    float linear_x;
    float angular_z;
};

struct StatePacket {
    int64_t left_ticks;
    int64_t right_ticks;
};
#pragma pack(pop)

class UdpRobotNode : public rclcpp::Node {
public:
    UdpRobotNode() : Node("udp_bridge_node"), io_context_(), socket_(io_context_, udp::endpoint(udp::v4(), 4211)) {

        // Настройка адреса ESP32 (проверь IP!)
        esp32_endpoint_ = udp::endpoint(boost::asio::ip::make_address("192.168.10.212"), 4210);

        // Издатель для JointStates
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Подписчик на Twist
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&UdpRobotNode::cmd_callback, this, std::placeholders::_1));

        // Таймер для чтения UDP (чтобы не блокировать основной поток)
        receive_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&UdpRobotNode::receive_udp_data, this));

        RCLCPP_INFO(this->get_logger(), "UDP Bridge Node started.");
    }

private:
    // Отправка команд на ESP32
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        CommandPacket pkt;
        pkt.linear_x = static_cast<float>(msg->linear.x);
        pkt.angular_z = static_cast<float>(msg->angular.z);

        socket_.send_to(boost::asio::buffer(&pkt, sizeof(pkt)), esp32_endpoint_);
    }

    // Чтение данных от ESP32
    void receive_udp_data() {
        try {
            if (socket_.available()) {
                StatePacket pkt;
                udp::endpoint sender_endpoint;
                size_t len = socket_.receive_from(boost::asio::buffer(&pkt, sizeof(pkt)), sender_endpoint);

                if (len == sizeof(StatePacket)) {
                    publish_joint_states(pkt.left_ticks, pkt.right_ticks);
                }
            }
        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "UDP Receive Error: %s", e.what());
        }
    }

    void publish_joint_states(int64_t left, int64_t right) {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"left_wheel_joint", "right_wheel_joint"};

        // Здесь мы просто передаем тики как позицию.
        // В будущем здесь нужно умножить на (2*PI / тиков_на_оборот)
        msg.position = {static_cast<double>(left), static_cast<double>(right)};

        joint_pub_->publish(msg);
    }

    // Boost Asio члены
    boost::asio::io_context io_context_;
    udp::socket socket_;
    udp::endpoint esp32_endpoint_;

    // ROS2 члены
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr receive_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpRobotNode>());
    rclcpp::shutdown();
    return 0;
}
