#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class MovementToWall : public rclcpp::Node
{
public:
	MovementToWall()
	: Node("movement_to_wall")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/robot/cmd_vel", 10);
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/depth/image", 10,
			std::bind(&MovementToWall::callback, this, _1));
	}
private:
	// publisher и subscription'ы
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	// Текущие данные с камеры глубины
	sensor_msgs::msg::Image current_data_;
	// Конкретно изображение с камеры глубины типа float
	float* current_data_float_;
	// Индекс пикселя, по которому ориентируемся
	int center_index_ = -1;
	// Указания к передвижению и 
	// флаг, определяющий, возможно ли это передвижение
	geometry_msgs::msg::Twist movement_;
	int have_to_move_ = 1;
	
	void callback(sensor_msgs::msg::Image msg)
	{
		current_data_ = msg;
		
		if (center_index_ == -1)
		{
			center_index_ = (current_data_.height / 2) * current_data_.width + 
							current_data_.width / 2;
		}
		
		// Преобразуем массив Image.data из типа uchar* к типу float*
		current_data_float_ = (float*)current_data_.data.data();
		
		have_to_move_ = current_data_float_[center_index_] > 0.3 ? 1 : 0;
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
