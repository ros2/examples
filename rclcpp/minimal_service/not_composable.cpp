#include "rclcpp/rclcpp.hpp"
#include "examples_rclcpp_minimal_service/srv/add_two_ints.hpp"

using Request  = examples_rclcpp_minimal_service::srv::AddTwoInts::Request;
using Response = examples_rclcpp_minimal_service::srv::AddTwoInts::Response;

void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Request> request,
    const std::shared_ptr<Response> response)
{
  (void)request_header;
  printf("request: %ld + %ld\n", request->a, request->b);
  response->sum = request->a + request->b;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_service");
  auto server =
      node->create_service<examples_rclcpp_minimal_service::srv::AddTwoInts>(
          "service", handle_service);

  rclcpp::spin(node);
  
  return 0;
}
