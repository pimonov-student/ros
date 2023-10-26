#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class MoveToGoal : public rclcpp::Node
{
public:
	MoveToGoal(int argc, char** argv)
	: Node("move_to_goal")
	{
		if (argc != 4)
		{
			std::cout << "Invalid arguments" << std::endl;
			std::exit(-1);
		}
		
		goal_.x = 	  std::stof(argv[1]);
		goal_.y = 	  std::stof(argv[2]);
		goal_.theta = std::stof(argv[3]);
		
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/turtle1/cmd_vel", 10);
		subscription_ = this->create_subscription<turtlesim::msg::Pose>(
			"/turtle1/pose", 10,
			std::bind(&MoveToGoal::pose_callback, this, _1));
	}
private:
	turtlesim::msg::Pose goal_;
	turtlesim::msg::Pose turtle_;
	
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
	
	int optimum_flag_ = 1;
	
	void pose_callback(const turtlesim::msg::Pose& message)
	{
		turtle_ = message;
		
		rclcpp::Rate loop_rate(10);
		
		geometry_msgs::msg::Twist twist;
		float distance;
		float angle;
		float angle_to_turn;
		
		distance = std::sqrt(std::pow(goal_.x - turtle_.x, 2) +
							 std::pow(goal_.y - turtle_.y, 2));
		angle = std::atan2((goal_.y - turtle_.y), (goal_.x - turtle_.x));
		
		angle_to_turn = angle - turtle_.theta;
		
		if (distance > 0.01)
		{
			if (optimum_flag_ && abs(angle_to_turn) > 0.001)
			{
				twist.angular.z = angle_to_turn;
				twist.linear.x = 0.1 * distance;
			}
			if (twist.linear.x == 0)
			{
				optimum_flag_ = 0;
				twist.linear.x = distance;
			}
		}
		else
		{
			if (abs(goal_.theta - turtle_.theta) > 0.01)
			{
				twist.angular.z = goal_.theta - turtle_.theta;
			}
			else
			{
				rclcpp::shutdown();
			}
		}
		
		RCLCPP_INFO(this->get_logger(), "%f, %f, %f", distance, angle, angle_to_turn);
		publisher_->publish(twist);
		
		loop_rate.sleep();
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MoveToGoal>(argc, argv));
	return 0;
}
