#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
#include "geometry_msgs/msg/twist.hpp"


class TwistPublisher : public rclcpp::Node
{
  public:
    TwistPublisher()
    : Node("TwistPublisher")
    {
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("topic1", 10);
        timer = this->create_wall_timer(
        500ms, std::bind(&TwistPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
       
        message.linear.x = 1;
        message.angular.z = 1.5;

        RCLCPP_INFO(this->get_logger(), "Published! temp: %f", message.angular.z);
        publisher -> publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPublisher>());
  rclcpp::shutdown();
  return 0;
}