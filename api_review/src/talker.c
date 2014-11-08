#include <api_review/rclc/rclc.h>
#include <api_review/rclc/node.h>

int main(int argc, char **argv)
{
  int ret = 0;
  ret = rclc_init(argc, argv);
  rclc_node_t node;
  ret = rclc_create_node("talker", &node);

  return 0;
}
