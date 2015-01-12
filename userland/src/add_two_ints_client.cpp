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
  auto response = std::make_shared<userland_msgs::AddTwoInts::Response>();
  request->a = 2;
  request->b = 3;

  auto rc = client->send_request(request, response);
  if(rc == 0)
  {
    std::cout << "Sum: " << response->sum << std::endl;
    return 0;
  } else
  {
    std::cerr << "Error receiving a response" << std::endl;
  }

  return rc;
}
