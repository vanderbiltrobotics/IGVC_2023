#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "Math.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

const double WHEEL_BASE = 1;
#define PI 3.14159265

class TwistSub_AckerPub : public rclcpp::Node
{
  public:
    TwistSub_AckerPub()
    : Node("TwistSub_AckerPub")
    {
        publisher = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("topic2", 1);

        subscription = this->create_subscription<geometry_msgs::msg::Twist>(
            "topic1", 10, std::bind(&TwistSub_AckerPub::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) 
    {
        double v = msg->linear.x;
        double steering = convertToSteeringAngle(v, msg->angular.z, WHEEL_BASE);

        auto message = ackermann_msgs::msg::AckermannDrive();
        message.steering_angle = steering;
        message.speed = v;
        RCLCPP_INFO(this->get_logger(), "Recieved! Speed: %f Steering angle: %f", message.speed, message.steering_angle);
        publisher -> publish(message);
    }

    double convertToSteeringAngle(double v, double omega, double wheelbase)
    {
        if (omega == 0 || v == 0){
            return 0;
        }
        
        double radius = v / omega;
        return atan(wheelbase / radius) * 180 / PI;
    }


    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistSub_AckerPub>());
  rclcpp::shutdown();
  return 0;
}