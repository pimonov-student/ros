#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/full_name_sum_service.hpp"

void full_name(const std::shared_ptr<service_full_name::srv::FullNameSumService::Request> request,
			   std::shared_ptr<service_full_name::srv::FullNameSumService::Response>      response)
{
  response->full_name = request->last_name + " " + request->name + " " + request->first_name;
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nLast name: %s\n" "Name: %s\n" "First name: %s",
                request->last_name.c_str(), request->name.c_str(), request->first_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->full_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("full_name_server");

  rclcpp::Service<service_full_name::srv::FullNameSumService>::SharedPtr service =
    node->create_service<service_full_name::srv::FullNameSumService>("SumFullName", &full_name);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to sum names");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
