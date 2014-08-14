#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "rclcpp/executor/SingleThreadExecutor.h"
#include "simple_msgs/AllPrimitiveTypes.h"

void callback(const simple_msgs::AllPrimitiveTypes *msg)
{
    std::cout << "Got message:" << std::endl;
    std::cout << "  my_bool: " << msg->my_bool << std::endl;
    std::cout << "  my_byte: " << msg->my_byte << std::endl;
    std::cout << "  my_char: " << msg->my_char << std::endl;
    std::cout << "  my_float32: " << msg->my_float32 << std::endl;
    std::cout << "  my_float64: " << msg->my_float64 << std::endl;
    std::cout << "  my_int8: " << msg->my_int8 << std::endl;
    std::cout << "  my_uint8: " << msg->my_uint8 << std::endl;
    std::cout << "  my_int16: " << msg->my_int16 << std::endl;
    std::cout << "  my_uint16: " << msg->my_uint16 << std::endl;
    std::cout << "  my_int32: " << msg->my_int32 << std::endl;
    std::cout << "  my_uint32: " << msg->my_uint32 << std::endl;
    std::cout << "  my_int64: " << msg->my_int64 << std::endl;
    std::cout << "  my_uint64: " << msg->my_uint64 << std::endl;
    std::cout << "  my_string: " << msg->my_string << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::Node *node = rclcpp::create_node();
  node->create_subscriber<simple_msgs::AllPrimitiveTypes>("topic_name", callback);

  rclcpp::executor::SingleThreadExecutor *executor = new rclcpp::executor::SingleThreadExecutor();

  executor->register_node(node);
  executor->exec();

  return 0;
}
