#include "rclcpp/Node.h"
#include "rclcpp/Publisher.h"
#include "rclcpp/executor/SingleThreadExecutor.h"

#include "simple_msgs/AllDynamicArrayTypes.h"
#include "simple_msgs/AllPrimitiveTypes.h"
#include "simple_msgs/AllStaticArrayTypes.h"
#include "simple_msgs/Nested.h"
#include "simple_msgs/String.h"
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

#define PRINT_STATIC_ARRAY_FIELD(FIELD_NAME, SIZE) \
  std::cout << "  " << #FIELD_NAME << "[" << SIZE << "]"; \
  for (size_t i = 0; i < SIZE; ++i) { \
    std::cout << " " << msg->FIELD_NAME[i]; \
  } \
  std::cout << std::endl;

void print_all_static_array(const simple_msgs::AllStaticArrayTypes *msg)
{
  std::cout << "Got message:" << std::endl;
  PRINT_STATIC_ARRAY_FIELD(my_bool, 6)
  PRINT_STATIC_ARRAY_FIELD(my_byte, 6)
  PRINT_STATIC_ARRAY_FIELD(my_char, 6)
  PRINT_STATIC_ARRAY_FIELD(my_float32, 6)
  PRINT_STATIC_ARRAY_FIELD(my_float64, 6)
  PRINT_STATIC_ARRAY_FIELD(my_int8, 6)
  PRINT_STATIC_ARRAY_FIELD(my_uint8, 6)
  PRINT_STATIC_ARRAY_FIELD(my_int16, 6)
  PRINT_STATIC_ARRAY_FIELD(my_uint16, 6)
  PRINT_STATIC_ARRAY_FIELD(my_int32, 6)
  PRINT_STATIC_ARRAY_FIELD(my_uint32, 6)
  PRINT_STATIC_ARRAY_FIELD(my_int64, 6)
  PRINT_STATIC_ARRAY_FIELD(my_uint64, 6)
  PRINT_STATIC_ARRAY_FIELD(my_string, 6)
}

#define PRINT_DYNAMIC_ARRAY_FIELD(FIELD_NAME) \
  std::cout << "  " << #FIELD_NAME << "[]"; \
  for (auto it : msg->FIELD_NAME) { \
    std::cout << " " << it; \
  } \
  std::cout << std::endl;

void print_all_dynamic_array(const simple_msgs::AllDynamicArrayTypes *msg)
{
  std::cout << "Got message:" << std::endl;
  PRINT_DYNAMIC_ARRAY_FIELD(my_bool)
  PRINT_DYNAMIC_ARRAY_FIELD(my_byte)
  PRINT_DYNAMIC_ARRAY_FIELD(my_char)
  PRINT_DYNAMIC_ARRAY_FIELD(my_float32)
  PRINT_DYNAMIC_ARRAY_FIELD(my_float64)
  PRINT_DYNAMIC_ARRAY_FIELD(my_int8)
  PRINT_DYNAMIC_ARRAY_FIELD(my_uint8)
  PRINT_DYNAMIC_ARRAY_FIELD(my_int16)
  PRINT_DYNAMIC_ARRAY_FIELD(my_uint16)
  PRINT_DYNAMIC_ARRAY_FIELD(my_int32)
  PRINT_DYNAMIC_ARRAY_FIELD(my_uint32)
  PRINT_DYNAMIC_ARRAY_FIELD(my_int64)
  PRINT_DYNAMIC_ARRAY_FIELD(my_uint64)
  PRINT_DYNAMIC_ARRAY_FIELD(my_string)
}

void print_nested(const simple_msgs::Nested *msg)
{
  std::cout << "Got message #" << msg->submsg.data << std::endl;
}

void print_string(const simple_msgs::String *msg)
{
  std::cout << "Got message: " << msg->data << std::endl;
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
  } else if (msg_arg == valid_message_args[2]) {
    subscribe<simple_msgs::AllStaticArrayTypes>(node, &print_all_static_array);
  } else if (msg_arg == valid_message_args[3]) {
    subscribe<simple_msgs::AllDynamicArrayTypes>(node, &print_all_dynamic_array);
  } else if (msg_arg == valid_message_args[4]) {
    subscribe<simple_msgs::Nested>(node, &print_nested);
  } else if (msg_arg == valid_message_args[5]) {
    subscribe<simple_msgs::String>(node, &print_string);
  } else {
    std::cerr << "unsupported '--msg' argument '" << msg_arg << "'" << std::endl;
    return 1;
  }

  rclcpp::executor::SingleThreadExecutor *executor = new rclcpp::executor::SingleThreadExecutor();

  executor->register_node(node);
  executor->exec();

  return 0;
}
