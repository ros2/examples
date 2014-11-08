typedef struct rclc_node_t
{
  char *name;
} rclc_node_t;

int
rclc_create_node(char *name, rclc_node_t *node);

int
rclc_destroy_node(rclc_node_t *node);
