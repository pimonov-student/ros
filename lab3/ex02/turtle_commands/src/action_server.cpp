#include <functional>
#include <memory>
#include <thread>

#include "message_turtle_commands/action/message_turtle_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

namespace turtle_action
{
// Класс action server'а
class TurtleActionServer : public rclcpp::Node
{
public:
	// Для удобства вставки в шаблоны
	using TurtleAction = message_turtle_commands::action::MessageTurtleCommands;
	using GoalHandleTurtleAction = rclcpp_action::ServerGoalHandle<TurtleAction>;
	
	// Конструктор класса, создает ноду "action_server"
	TurtleActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
	: Node("action_server", options)
	{
		using namespace std::placeholders;
		// Вот он server, в нем callback функции (определены будут потом)
		this->action_server_ = rclcpp_action::create_server<TurtleAction>(
			this,
			"turtle_action",
			std::bind(&TurtleActionServer::handle_goal, this, _1, _2),
			std::bind(&TurtleActionServer::handle_cancel, this, _1),
			std::bind(&TurtleActionServer::handle_accepted, this, _1));
		
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"turtle1/cmd_vel", 10);
		subscription_ = this->create_subscription<turtlesim::msg::Pose>(
			"turtle1/pose", 10,
			std::bind(&TurtleActionServer::pose_callback, this, _1));
	}
private:
	// server
	rclcpp_action::Server<TurtleAction>::SharedPtr action_server_;
	// publisher и subscription, нужны для связи с топиками cmd_vel и pose
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
	
	// Позиции черепашки (текущая и начальная)
	turtlesim::msg::Pose pose_curr_;
	turtlesim::msg::Pose pose_init_;
	
	
	// Обработчик запроса на выполнение goal'а
	rclcpp_action::GoalResponse handle_goal(
		const rclcpp_action::GoalUUID& uuid,
		std::shared_ptr<const TurtleAction::Goal> goal)
	{
		RCLCPP_INFO(this->get_logger(), "Recieved goal request with command %s", goal->command.c_str());
		(void)uuid;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
	
	// Обработчик запроса на прерывание выполнения goal'а
	rclcpp_action::CancelResponse handle_cancel(
		const std::shared_ptr<GoalHandleTurtleAction> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal");
		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	// Обработчик успешного получения запроса на выполнение goal'а
	void handle_accepted(const std::shared_ptr<GoalHandleTurtleAction> goal_handle)
	{
		using namespace std::placeholders;
		std::thread{std::bind(&TurtleActionServer::execute, this, _1), goal_handle}.detach();
	}
	
	
	// Callback функция, текущее местоположение черепашки
	void pose_callback(const turtlesim::msg::Pose& message)
	{
		pose_curr_ = message;
	}
	
	// Основная функция
	void execute(const std::shared_ptr<GoalHandleTurtleAction> goal_handle)
	{
		RCLCPP_INFO(this->get_logger(), "Executing goal");
		
		// Получаем goal
		const auto goal = goal_handle->get_goal();
		// result и feedback
		auto result = std::make_shared<TurtleAction::Result>();
		auto feedback = std::make_shared<TurtleAction::Feedback>();
		// Задаем частоту отправки feedback'а
		rclcpp::Rate loop_rate(10);
		
		// Twist сообщение
		geometry_msgs::msg::Twist twist_message;
		// Одометрия (feedback)
		float odom = 0;
		// Обновляем начальную позицию черепашки
		pose_init_ = pose_curr_;
		
		
		// Смотрим, что за команда и выполняем (или не выполняем...)
		// Все значения берем из goal'а
		if (goal->command == "forward")
		{
			twist_message.linear.x = goal->s;
		}
		else if (goal->command == "turn_right")
		{
			twist_message.angular.z = -(goal->angle * 3.14 / 180);
		}
		else if (goal->command == "turn_left")
		{
			twist_message.angular.z = goal->angle * 3.14 / 180;
		}
		
		// Публикуем в cmd_vel
		publisher_->publish(twist_message);
		
		// Если командой было движение forward, имеет смысл отправлять одометрию (feedback)
		if (goal->command == "forward")
		{
			// Флаг для проверки, движется ли черепаха
			int flag = 1;
			// Пока черепашка движется и все хорошо,
			// публикуем feedback
			while(flag && rclcpp::ok())
			{
				// Тут обрабатываем, если черепашке вдруг сказано остановиться
				if (goal_handle->is_canceling())
				{
					result->result = false;
					goal_handle->canceled(result);
					RCLCPP_INFO(this->get_logger(), "Goal cancelled");
					publisher_->publish(geometry_msgs::msg::Twist());
					break;
				}
				
				// Изначально скорость будет 0, фиксируем когда станет ненулевой
				if (flag == 1 && pose_curr_.linear_velocity != 0)
				{
					flag = 2;
				}
				// Затем фиксируем, когда скорость вновь стала нулевой
				if (flag == 2 && pose_curr_.linear_velocity == 0)
				{
					flag = 0;
				}
				
				// Считаем одометрию как длину вектора (pose_init_, pose_curr_)
				odom = std::sqrt(pow(pose_curr_.x - pose_init_.x, 2) + pow(pose_curr_.y - pose_init_.y, 2));
				// Обновляем и отправляем feedback
				feedback->odom = odom;
				goal_handle->publish_feedback(feedback);
				RCLCPP_INFO(this->get_logger(), "Turtle moved at %f m.", odom);
				
				loop_rate.sleep();
			}
		}
		// Если же командой был поворот,
		// есть смысл попридержать execute функцию, пока черепашка не докрутится
		else
		{
			int flag = 1;
			while(flag && rclcpp::ok())
			{
				if (flag == 1 && pose_curr_.angular_velocity != 0)
				{
					flag = 2;
				}
				if (flag == 2 && pose_curr_.angular_velocity == 0)
				{
					flag = 0;
				}
				
				RCLCPP_INFO(this->get_logger(), "Turtle is spinning");
				loop_rate.sleep();
			}
		}
		
		// Заканчиваем goal
		if (rclcpp::ok())
		{
			result->result = true;
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Goal succeeded");
		}
	}
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(turtle_action::TurtleActionServer)
