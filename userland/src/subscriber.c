#include <stdio.h>

#include <rclc/rclc.h>

#include <simple_msgs/Int32-c.h>

void callback(const void * void_message)
{
  const simple_msgs__Int32 * message = (const simple_msgs__Int32*)void_message;
  printf("Got number: %d\n", message->data);
}

int main(int argc, char **argv)
{
  rclc_init(argc, argv);

  rclc_node_t * node = rclc_create_node("subscriber");

  rclc_subscription_t * subscription = rclc_create_subscription(
    node, ROSIDL_GET_TYPE_SUPPORT(simple_msgs, Int32), "chatter", callback, 7
  );

  rclc_spin_node(node);

  rclc_destroy_subscription(subscription);
  rclc_destroy_node(node);

  return 0;
}
