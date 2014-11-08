#include <memory>
#include <string>

namespace rclcpp
{
namespace node
{

struct Node
{
  Node(std::string name);

  bool
  is_valid();

private:
  std::string name_;

};

typedef std::unique_ptr<Node> NodePtr;
typedef std::shared_ptr<Node> NodeSharedPtr;

NodePtr
create_node(std::string name)
{
  return NodePtr(new Node(name));
}

} /* namespace node */
} /* namespace rclcpp */