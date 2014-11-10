#include <rclc/rclc.h>
#include <rclc/node.h>

int main(int argc, char **argv)
{
  int ret = 0;
  ret = rclc_init(argc, argv);
  rclc_node_t * node;
  node = rclc_create_node("talker");

  return 0;
}
