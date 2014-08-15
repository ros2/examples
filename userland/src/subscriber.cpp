#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "rclcpp/executor/SingleThreadExecutor.h"

#include "simple_msgs/AllPrimitiveTypes.h"
#include "simple_msgs/Uint32.h"

#include "userland/command_line_arguments.h"


template<typename T>
void subscribe(rclcpp::Node* node, void (*print_data_func)(const T*))
{
  node->create_subscriber<T>("topic_name", print_data_func);
}

void print_counter_data(const simple_msgs::Uint32 *msg)
{
  std::cout << "Got message #" << msg->data << std::endl;
}

void print_all_primitive_data(const simple_msgs::AllPrimitiveTypes *msg)
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
  if (has_argument(argv, argv + argc, "--help")) {
    std::cout << "usage:" << argv[0] << std::endl;
    print_message_usage();
    return 0;
  }

  rclcpp::Node * node = rclcpp::create_node();

  const std::string msg_arg = get_named_argument(argv, argv + argc, "--msg", valid_message_args[0]);
  if (msg_arg == valid_message_args[0]) {
    subscribe<simple_msgs::Uint32>(node, &print_counter_data);
  } else if (msg_arg == valid_message_args[1]) {
    subscribe<simple_msgs::AllPrimitiveTypes>(node, &print_all_primitive_data);
  } else {
    std::cerr << "unsupported '--msg' argument '" << msg_arg << "'" << std::endl;
    return 1;
  }

  rclcpp::executor::SingleThreadExecutor *executor = new rclcpp::executor::SingleThreadExecutor();

  executor->register_node(node);
  executor->exec();

  return 0;
}
