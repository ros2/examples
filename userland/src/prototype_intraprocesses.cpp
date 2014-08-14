#include <iostream>
#include <sys/time.h>
#include <chrono>
#include <thread>

#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "simple_msgs/Intraprocess.h"
#include "rclcpp/executor/SingleThreadExecutor.h"

struct Intraprocess_t{
  std::string* data; // data 
  int count; // # of receivers
};

void decrement(Intraprocess_t* meta)
{
  (meta->count)--;
  if (meta->count == 0) {
    std::cout << "------ remove the object!"  << std::endl;
    delete meta->data;
  }   
}

void callback(const simple_msgs::Intraprocess *msg)
{
  Intraprocess_t* meta = (Intraprocess_t *) msg->ptr;
  std::cout << "------ Got message: " <<  *(meta->data) << std::endl;
  decrement(meta);
}

void monitor(std::string* s)
{
  while (1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "------ monitor string:" << *s << std::endl;  
  }
}

void launch_subscriber(void)
{
  std::cout << "------ Creating subscriber:" << std::endl;
  rclcpp::Node *node = rclcpp::create_node();
  node->create_subscriber<simple_msgs::Intraprocess>("intraprocess", callback);

  rclcpp::executor::SingleThreadExecutor *executor = new rclcpp::executor::SingleThreadExecutor();
  executor->register_node(node);
  std::cout << "------ Launching subscriber:" << std::endl;
  executor->exec();
}

void launch_publisher(Intraprocess_t* s)
{
  std::cout << "------ Creating publisher:" << std::endl;
  rclcpp::Node* n = rclcpp::create_node();
  rclcpp::Publisher<simple_msgs::Intraprocess>* p = n->create_publisher<simple_msgs::Intraprocess>("intraprocess");
  simple_msgs::Intraprocess ros_msg;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait one second for the publisher to set up

  int number_of_msgs = 1;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < number_of_msgs; ++i) {    
    ros_msg.ptr = (uint64_t)s;
    p->publish(ros_msg);
    // if (i % 100000 == 0) {
      std::cout << "------ published ros pointer to Intraprocess_t " << i << std::endl;
    // }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  auto end = std::chrono::steady_clock::now();

  //std::cout << (end - start).count() << std::endl;  
}

int main(int argc, char** argv)
{
  std::string* s = new std::string("Message sent through ROS"); 
  Intraprocess_t meta;
  meta.data = s;
  meta.count = 1;

  // monitor
  std::thread monitor_thread (monitor, s);
  // publisher
  std::thread publisher_thread (launch_publisher, &meta);
  // subscriber
  launch_subscriber();
  publisher_thread.join();

  return 0;
}
