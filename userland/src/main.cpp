#include <iostream>
#include <sys/time.h>

#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "std_msgs/Int32.h"


int main(int argc, char** argv)
{
  rclcpp::Node* n = create_node();
  rclcpp::Publisher<std_msgs::Int32>* p = n->create_publisher<std_msgs::Int32>("topic_name");
  std_msgs::Int32 ros_msg;

  int number_of_msgs = 1000000;

  timespec start;
  clock_gettime(CLOCK_REALTIME, &start);

  for (int i = 1; i < number_of_msgs; ++i) {
    ros_msg.data = i;
    p->publish(ros_msg);
    if (i % 100000 == 0) {
      std::cout << "published ros msg #" << i << std::endl;
    }
  }

  timespec end;
  clock_gettime(CLOCK_REALTIME, &end);

  std::cout << (end.tv_sec - start.tv_sec) << "." << (end.tv_nsec - start.tv_nsec) << std::endl;

  return 0;
}
