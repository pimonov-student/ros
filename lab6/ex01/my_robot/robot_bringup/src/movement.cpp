#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class Movement : public rclcpp::Node
{
public:
	Movement()
	: Node("movement")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/robot/cmd_vel", 10);
		text_subscription_ = this->create_subscription<std_msgs::msg::String>(
			"cmd_text", 10,
			std::bind(&Movement::text_callback, this, _1));
		pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/robot/odom", 10,
			std::bind(&Movement::pose_callback, this, _1));
	}
private:
	// publisher и subscription'ы
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_subscription_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
	// Команда
	std_msgs::msg::String command_;
	// Сообщение, которое мы отправим в /cmd_vel
	geometry_msgs::msg::Twist twist_;
	// Текущие, начальные и целевые данные одометрии
	nav_msgs::msg::Odometry odom_curr_;
	nav_msgs::msg::Odometry odom_init_;
	nav_msgs::msg::Odometry odom_goal_;
	// Флаг действий,
	// 0 - ничего не надо делать
	int action_flag_ = 0;
	
	// callback для текста
	void text_callback(const std_msgs::msg::String& text_msg)
	{
		// Получаем команду
		command_ = text_msg;
		
		// Варианты комманд
		if (command_.data == "forward")
		{
			twist_.linear.x = 0.2;
		}
		else if (command_.data == "backward")
		{
			twist_.linear.x = -0.2;
		}
		else if (command_.data == "right")
		{
			twist_.angular.z = -0.2;
		}
		else if (command_.data == "left")
		{
			twist_.angular.z = 0.2;
		}
		
		// 1 - получена команда,
		// необходимо:
		// зафиксировать "начальное" значение одометрии;
		// вычислить "целевое" значение одометрии
		action_flag_ = 1;
	}
	
	void pose_callback(const nav_msgs::msg::Odometry& odom_msg)
	{
		odom_curr_ = odom_msg;
		
		if (action_flag_ == 1)
		{
			// Фиксируем текущее значение одометрии как начальное
			odom_init_ = odom_curr_;
			odom_goal_ = odom_init_;
			
			// Целевое значение одометрии вычисляется только при поворотах,
			// так как мы можем рассчитать положение после поворота,
			// но после движения вперед / назад проще
			// отслеживать расстояние динамически
			if (command_.data == "right" || command_.data == "left")
			{
				// Вспомогательные переменные для поворотов
				geometry_msgs::msg::Quaternion init_q_msg;
				geometry_msgs::msg::Quaternion goal_q_msg;
				tf2::Quaternion init_q_tf2;
				tf2::Quaternion rotation_q_tf2;
				tf2::Quaternion goal_q_tf2;
				
				// Получаем начальное положение
				init_q_msg = odom_goal_.pose.pose.orientation;
				// Переводим его в tf2 формат
				tf2::fromMsg(init_q_msg, init_q_tf2);
			
				// Задаем кватернион поворота на 90 градусов
				command_.data == "right" ? rotation_q_tf2.setRPY(0.0, 0.0, -1.570796) :
										   rotation_q_tf2.setRPY(0.0, 0.0, 1.570796);
				// Поворачиваем, нормализуем и преобразуем в msg формат
				goal_q_tf2 = rotation_q_tf2 * init_q_tf2;
				goal_q_tf2.normalize();
				goal_q_msg = tf2::toMsg(goal_q_tf2);
				
				// Задаем целевое положение
				odom_goal_.pose.pose.orientation = goal_q_msg;
			}
			
			// 2 - целевая одометрия вычислена,
			// теперь необходимо двигаться
			action_flag_ = 2;
		}
		
		if (action_flag_ == 2)
		{
			if (command_.data == "forward" || command_.data == "backward")
			{
				// Расстояние между начальным значением одометрии и текущим
				float distance = std::sqrt(std::pow(odom_curr_.pose.pose.position.x -
													odom_init_.pose.pose.position.x, 2) +
										   std::pow(odom_curr_.pose.pose.position.y -
									 				odom_init_.pose.pose.position.y, 2));
				
				// Если дистанция превысила 1, команда выполнена
				if(distance >= 1.0)
				{
					action_flag_ = 0;
					twist_ = geometry_msgs::msg::Twist();
				}
			}
			
			if (command_.data == "right" || command_.data == "left")
			{
				// Разница между значениями z-компоненты кватеринонов,
				// отвечающих за orientation
				float q_difference = std::abs(odom_goal_.pose.pose.orientation.z -
											  odom_curr_.pose.pose.orientation.z);
				
				// Если разница достаточно мала, команда выполнена
				if (q_difference <= 0.001)
				{
					action_flag_ = 0;
					twist_ = geometry_msgs::msg::Twist();
				}
			}
			
			publisher_->publish(twist_);
		}
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Movement>());
	rclcpp::shutdown();
	return 0;
}

