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

void add(const std::shared_ptr<userland_msgs::AddTwoIntsRequestWithHeader> req,
         std::shared_ptr<userland_msgs::AddTwoIntsResponse> res)
{
  std::cout << "Incoming request" << std::endl;
  std::cout << "a: " << req->request.a << " b: " << req->request.b << std::endl;
  res->sum = req->request.a + req->request.b;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_server");

  node->create_service<userland_msgs::AddTwoInts>("add_two_ints", add);

  rclcpp::spin(node);

  return 0;
}
