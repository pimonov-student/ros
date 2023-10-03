#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MyNode : public rclcpp::Node
{
public:
	MyNode()
	: Node("text_to_cmd_vel")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/turtle1/cmd_vel", 10);
		subscription_ = this->create_subscription<std_msgs::msg::String>(
			"cmd_text", 10,
			std::bind(&MyNode::topic_callback, this, _1));
	}
private:
	void topic_callback(const std_msgs::msg::String & msg)
	{
		geometry_msgs::msg::Twist twist;
		if (msg.data == "turn_right")
		{
			twist.angular.z = -1.57;
		}
		else if (msg.data == "turn_left")
		{
			twist.angular.z = 1.57;
		}
		else if (msg.data == "move_forward")
		{
			twist.linear.x = 1.0;
		}
		else if (msg.data == "move_backward")
		{
			twist.linear.x = -1.0;
		}
		
		publisher_->publish(twist);
	}
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MyNode>());
	rclcpp::shutdown();
	return 0;
}
