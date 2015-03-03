#include <rclc/rclc.h>

#include <simple_msgs/Int32-c.h>

int main(int argc, char **argv)
{
  rclc_init(argc, argv);

  rclc_node_t * node = rclc_create_node("publisher");

  rclc_publisher_t * publisher = rclc_create_publisher(
    node, ROSIDL_GET_TYPE_SUPPORT(simple_msgs, Int32), "chatter", 7
  );

  size_t count = 0;
  while (rclc_ok())
  {
    simple_msgs__Int32 msg;
    msg.data = count++;
    rclc_publish(publisher, &msg);
    rclc_sleep_ms(1000);
  }

  rclc_destroy_publisher(publisher);
  rclc_destroy_node(node);

  return 0;
}
