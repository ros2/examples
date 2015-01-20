#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/AllBuiltinTypes.h>
#include <simple_msgs/AllDynamicArrayTypes.h>
#include <simple_msgs/AllPrimitiveTypes.h>
#include <simple_msgs/AllStaticArrayTypes.h>
#include <simple_msgs/Nested.h>
#include <simple_msgs/String.h>
#include <simple_msgs/Uint32.h>

#include <userland_msgs/AddTwoInts.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client");

  auto client = node->create_client<userland_msgs::AddTwoInts>("add_two_ints");
  auto request = std::make_shared<userland_msgs::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  auto f = client->async_send_request(request);

  std::future_status status;
  do
  {
    rclcpp::spin_some(node);
    status = f.wait_for(std::chrono::milliseconds(100));
  } while(status != std::future_status::ready && rclcpp::ok());

  if(std::future_status::ready == status)
  {
    std::cout << "FUTURE READY" << std::endl;
    std::cout << f.get()->sum << std::endl;
  }

  client->async_send_request(
    request,
    [] (rclcpp::client::Client<userland_msgs::AddTwoInts>::SharedFuture cb_f) {
      std::cout << "CALLBACK" << std::endl;
      std::cout << cb_f.get()->sum << std::endl;
  });
  rclcpp::spin(node);

  return 0;
}
