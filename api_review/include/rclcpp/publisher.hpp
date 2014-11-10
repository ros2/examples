#include <memory>
#include <string>
#include <rclcpp/node.hpp>

namespace rclcpp
{
namespace publisher
{

struct Publisher
{
  explicit Publisher(node::NodeHandleSharedPtr &node_handle,
                     std::string topic,
                     std::size_t queue_size)
  : 
  {}
};

typedef std::unique_ptr<Publisher> PublisherPtr;
typedef std::shared_ptr<Publisher> PublisherSharedPtr;

struct PublisherHandle
{
  explicit PublisherHandle(node::NodeHandleSharedPtr &node_handle,
                           std::string topic,
                           std::size_t queue_size)
  : publisher_(new Publisher(node_handle, topic, queue_size))
  {}

private:
  PublisherSharedPtr publisher_;

};

typedef std::unique_ptr<PublisherHandle> PublisherHandlePtr;

} /* namespace publisher */
} /* namespace rclcpp */
