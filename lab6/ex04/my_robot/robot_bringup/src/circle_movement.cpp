#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CircleMovement : public rclcpp::Node
{
public:
	CircleMovement()
	: Node("circle_movement")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/robot/cmd_vel", 10);
		timer_ = this->create_wall_timer(
			500ms, std::bind(&CircleMovement::timer_callback, this));
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	
	void timer_callback()
	{
		geometry_msgs::msg::Twist twist;
		
		twist.linear.x = 0.2;
		twist.angular.z = 0.5;
		
		publisher_->publish(twist);
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CircleMovement>());
	rclcpp::shutdown();
	return 0;
}
