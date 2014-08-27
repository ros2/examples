#include <functional>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/String.h>

void on_message(const simple_msgs::String::ConstPtr &msg)
{
  std::cout << "I heard [" << msg->data << "]" << std::endl;
}

void on_timer(rclcpp::Publisher::SharedPtr &publisher, int &i)
{
  simple_msgs::String::Ptr msg;
  msg->data = "Hello World: " + std::to_string(i++);
  if (publisher) {
    publisher->publish(msg);
  } else {
    std::cerr << "Invalid publisher!" << std::endl;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node1 = rclcpp::Node::make_shared("node1");
  auto node2 = rclcpp::Node::make_shared("node2");

  auto publisher = node1->create_publisher<simple_msgs::String>("chatter", 7);
  auto subscription = \
    node2->create_publisher<simple_msgs::String>("chatter", 7, on_message);

  int i = 0;
  using std::placeholders::_1;
  auto timer_bound = std::bind(on_timer, _1, std::ref(publisher), std::ref(i));
  auto timer = node1->create_timer(0.5, timer_bound);

  auto executor = rclcpp::executors::MultiThreadedExecutor();

  executor.register_node(node1);
  executor.register_node(node2);

  executor.exec();

  return 0;
}
