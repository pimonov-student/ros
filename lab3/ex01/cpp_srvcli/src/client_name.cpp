#include "rclcpp/rclcpp.hpp"
#include "service_full_name/srv/full_name_sum_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: full_name_client last_name name first_name");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("full_name_client");
  rclcpp::Client<service_full_name::srv::FullNameSumService>::SharedPtr client =
    node->create_client<service_full_name::srv::FullNameSumService>("SumFullName");

  auto request = std::make_shared<service_full_name::srv::FullNameSumService::Request>();
  request->last_name = argv[1];
  request->name = argv[2];
  request->first_name = argv[3];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full name: %s", result.get()->full_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service SumFullName");
  }

  rclcpp::shutdown();
  return 0;
}
