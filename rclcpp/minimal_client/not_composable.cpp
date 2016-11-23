#include "rclcpp/rclcpp.hpp"
#include "examples_rclcpp_minimal_service/srv/add_two_ints.hpp"

using Service = examples_rclcpp_minimal_service::srv::AddTwoInts;
using Request = Service::Request;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("minimal_subscriber");
  auto client = node->create_client<Service>("service");
  while (!client->wait_for_service(1_s)) {
    if (!rclcpp::ok()) {
      printf("client interrupted while waiting for service to appear.\n");
      return 1;
    }
    printf("waiting for service to appear...\n");
  }
  auto request = std::make_shared<Request>();
  request->a = 41;
  request->b =  1;
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    printf("service call failed :(\n");
    return 1;
  }
  auto result = result_future.get();
  printf("result of sending service request for %ld + %ld = %ld\n",
         request->a, request->b, result->sum);
  return 0;
}
