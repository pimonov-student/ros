#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "message_turtle_commands/action/message_turtle_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace turtle_action
{
// Класс action client'а
class TurtleActionClient : public rclcpp::Node
{
public:
	// Для удобства вставки в шаблоны
	using TurtleAction = message_turtle_commands::action::MessageTurtleCommands;
	using GoalHandleTurtleAction = rclcpp_action::ClientGoalHandle<TurtleAction>;
	
	// Конструктор класса, создает ноду "action_client"
	TurtleActionClient(
		std::vector<message_turtle_commands::action::MessageTurtleCommands::Goal> goals,
		const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
	: Node("action_client", options)
	{
		// Вот он client
		this->action_client_ = rclcpp_action::create_client<TurtleAction>(
			this,
			"turtle_action");
		
		// Если запустили client, а server'а нет
		if (!this->action_client_->wait_for_action_server())
		{
			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
			rclcpp::shutdown();
		}
		
		// Копируем входные goal'ы в ноду
		this->goals_ = goals;
		// Первой обработаем ноду номер 0 (она же первая...)
		this->curr_goal_ptr_ = 0;
		// Отправляем goal
		this->send_goal(this->goals_.at(this->curr_goal_ptr_));
	}
	
	// Функция, отправляет текущий goal
	void send_goal(message_turtle_commands::action::MessageTurtleCommands::Goal goal)
	{
		using namespace std::placeholders;
		
		// callback функции
		auto send_goal_options = rclcpp_action::Client<TurtleAction>::SendGoalOptions();
		send_goal_options.goal_response_callback = 
			std::bind(&TurtleActionClient::goal_response_callback, this, _1);
		send_goal_options.feedback_callback = 
			std::bind(&TurtleActionClient::feedback_callback, this, _1, _2);
		send_goal_options.result_callback =
			std::bind(&TurtleActionClient::result_callback, this, _1);
		
		// Непосредственно отправляем goal
		RCLCPP_INFO(this->get_logger(), "Sending goal");
		this->action_client_->async_send_goal(goal, send_goal_options);
	}
private:
	// action client
	rclcpp_action::Client<TurtleAction>::SharedPtr action_client_;
	// Вектоор наших goal'ов
	std::vector<message_turtle_commands::action::MessageTurtleCommands::Goal> goals_;
	// "Указатель" на текущий goal (его номер)
	size_t curr_goal_ptr_;
	
	// Обработчик ответа на запрос на выполнение goal'а
	void goal_response_callback(const GoalHandleTurtleAction::SharedPtr& goal_handle)
	{
		if (!goal_handle)
		{
			RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
		}
	}
	
	// Обработчик feedback'а
	void feedback_callback(
		GoalHandleTurtleAction::SharedPtr,
		const std::shared_ptr<const TurtleAction::Feedback> feedback)
	{
		RCLCPP_INFO(this->get_logger(), "Got: turtle moved at %f m.", feedback->odom);
	}
	
	// Обработчик результата
	void result_callback(const GoalHandleTurtleAction::WrappedResult& result)
	{
		switch(result.code)
		{
			case rclcpp_action::ResultCode::SUCCEEDED:
				break;
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
				return;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_ERROR(this->get_logger(), "Goal was cancelled");
				return;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				return;
		}
		RCLCPP_INFO(this->get_logger(), "Result: %s", result.result ? "true" : "false");
		
		// Будем обрабатывать следующий goal
		this->curr_goal_ptr_++;
		// Если он есть, конечно
		if (this->curr_goal_ptr_ < this->goals_.size())
		{
			this->send_goal(this->goals_.at(this->curr_goal_ptr_));
		}
		// Иначе отключаем ноду
		else
		{
			rclcpp::shutdown();
		}
	}
};
}

//RCLCPP_COMPONENTS_REGISTER_NODE(turtle_action::TurtleActionClient)

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	
	auto goal_1 = message_turtle_commands::action::MessageTurtleCommands::Goal();
	goal_1.command = "forward";
	goal_1.s = 2.0;
	goal_1.angle = 0;
	
	auto goal_2 = message_turtle_commands::action::MessageTurtleCommands::Goal();
	goal_2.command = "turn_right";
	goal_2.s = 1.0;
	goal_2.angle = 90;
	
	auto goal_3 = message_turtle_commands::action::MessageTurtleCommands::Goal();
	goal_3.command = "forward";
	goal_3.s = 2.0;
	goal_3.angle = 0;
	
	// Наши goal'ы
	auto goals = std::vector<message_turtle_commands::action::MessageTurtleCommands::Goal>();
	goals.push_back(goal_1);
	goals.push_back(goal_2);
	goals.push_back(goal_3);
	
	// client
	auto client = std::make_shared<turtle_action::TurtleActionClient>(goals);
	
	rclcpp::spin(client);
	
	return 0;
}
