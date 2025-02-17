#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

class DriveNode : public rclcpp::Node
{
public:
    DriveNode() : Node("drive_node")
    {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&DriveNode::publish_drive_command, this));
    }

private:
    void publish_drive_command()
    {
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.header.stamp = this->now();
        message.drive.speed = 1.0; // constant speed
        message.drive.steering_angle = 30.0; // going straight

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing drive command: speed %f, steering_angle %f", message.drive.speed, message.drive.steering_angle);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
