#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class MovementToWall : public rclcpp::Node
{
public:
	MovementToWall()
	: Node("movement_to_wall")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/robot/cmd_vel", 10);
		subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/robot/scan", 10,
			std::bind(&MovementToWall::callback, this, _1));
	}
private:
	// publisher и subscription'ы
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
	// Текущие данные с лидара
	sensor_msgs::msg::LaserScan current_data_;
	// Указания к передвижению и 
	// флаг, определяющий, возможно ли это передвижение
	geometry_msgs::msg::Twist movement_;
	int have_to_move_ = 1;
	
	void callback(sensor_msgs::msg::LaserScan msg)
	{
		current_data_ = msg;
		
		have_to_move_ = current_data_.ranges[179] > 0.3 ? 1 : 0;
		movement_.linear.x = have_to_move_ ? 0.2 : 0;
		
		publisher_->publish(movement_);
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MovementToWall>());
	rclcpp::shutdown();
	return 0;
}
