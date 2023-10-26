#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

// В классе все необходимое для движения
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
		
		// Координаты pose точки, в которую хотим приехать
		goal_.x = 	  std::stof(argv[1]);
		goal_.y = 	  std::stof(argv[2]);
		goal_.theta = std::stof(argv[3]);
		
		// Публикуем в /turtle1/cmd_vel, подписываемся на /turtle1/pose
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/turtle1/cmd_vel", 10);
		subscription_ = this->create_subscription<turtlesim::msg::Pose>(
			"/turtle1/pose", 10,
			std::bind(&MoveToGoal::pose_callback, this, _1));
	}
private:
	// pose назначения
	turtlesim::msg::Pose goal_;
	// Текущий pose черепашки
	turtlesim::msg::Pose turtle_;
	
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
	
	// Флаг, необходимый для ускорения движения черепашки
	int optimum_flag_ = 1;
	
	// Наш основной callback, в нем все действия
	void pose_callback(const turtlesim::msg::Pose& message)
	{
		// Обновляем хранящиеся в ноде координаты черепашки
		turtle_ = message;
		
		// Небольшая задержка, ускоряет работу и уменьшает лог
		rclcpp::Rate loop_rate(3);
		
		// Указания к передвиженю
		geometry_msgs::msg::Twist twist;
		// Дистанция между черепашкой и точкой назначения
		float distance;
		// Угол, в направлении которого будет точка назначения
		float angle;
		// Насколько надо повернуть черепашку,
		// чтоб она смотрела в направлении точки назначения
		float angle_to_turn;
		
		distance = std::sqrt(std::pow(goal_.x - turtle_.x, 2) +
							 std::pow(goal_.y - turtle_.y, 2));
		angle = std::atan2((goal_.y - turtle_.y), (goal_.x - turtle_.x));
		angle_to_turn = angle - turtle_.theta;
		
		// Пока не доехали в точку назначения
		if (distance > 0.01)
		{
			// Принцип работы флага:
			// Изначально на линейную скорость черепашки наложено ограничение,
			// наша основная задача: "навестись" на точку назначения
			if (optimum_flag_ && abs(angle_to_turn) > 0.001)
			{
				twist.angular.z = angle_to_turn;
				twist.linear.x = 0.1 * distance;
			}
			// Если черепашка навелась достаточно точно,
			// мы можем убрать ограничения на линейную скорость,
			// так как по сути нам остается только по прямой доехать до точки
			if (twist.linear.x == 0)
			{
				optimum_flag_ = 0;
				twist.linear.x = distance;
			}
			// Логируем оставшееся расстояние до точки назначения
			RCLCPP_INFO(this->get_logger(), "Distance to point: %f", distance);
		}
		// После того как доехали в точку назначения
		else
		{
			// Крутимся на месте до тех пор, пока черепашка не заимеет
			// theta, которое мы передавали изначально
			if (abs(goal_.theta - turtle_.theta) > 0.01)
			{
				twist.angular.z = goal_.theta - turtle_.theta;
			}
			// Как только докрутились, отключаем ноду
			else
			{
				rclcpp::shutdown();
			}
			// Логируем, сколько осталось докрутиться в радианах
			RCLCPP_INFO(this->get_logger(), "Angle to turn: %f", goal_.theta - turtle_.theta);
		}
		
		// Публикуем итоговые указания к перемещению
		publisher_->publish(twist);
		// Приостанавливаемся
		loop_rate.sleep();
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MoveToGoal>(argc, argv));
	return 0;
}
