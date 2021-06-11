#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int32_t main(const int32_t argc, char ** const argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("wait_set_example_node");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) {assert(false);};

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

  auto sub1 = node->create_subscription<std_msgs::msg::String>("a11", qos, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>("a22", qos, do_nothing);
  auto sub3 = node->create_subscription<std_msgs::msg::String>("a33", qos, do_nothing);

  std_msgs::msg::String msg1;
  msg1.data = "Hello, world!";

  std_msgs::msg::String msg2;
  msg2.data = "Hello, world!";

  std_msgs::msg::String msg3;
  msg3.data = "Hello, world!";

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(sub1);
  wait_set.add_subscription(sub2);
  wait_set.add_subscription(sub3);

  rclcpp::Node thread_node("thread");
  const auto pub1 =
    thread_node.create_publisher<std_msgs::msg::String>("a11", qos);
  const auto pub2 =
    thread_node.create_publisher<std_msgs::msg::String>("a22", qos);
  const auto pub3 =
    thread_node.create_publisher<std_msgs::msg::String>("a33", qos);

  // use transient_local ?
  /*
  pub1->wait_for_matched(1U, std::chrono::milliseconds(500U));
  pub2->wait_for_matched(1U, std::chrono::milliseconds(500U));
  pub3->wait_for_matched(1U, std::chrono::milliseconds(500U));

  sub1->wait_for_matched(1U, std::chrono::milliseconds(500U));
  sub2->wait_for_matched(1U, std::chrono::milliseconds(500U));
  sub3->wait_for_matched(1U, std::chrono::milliseconds(500U));
*/

  auto thread = std::thread(
    [msg1, msg2, msg3, pub1, pub2, pub3]() {
      pub2->publish(msg2);
      std::cout << "Publishing msg2: " << msg2.data << std::endl;
      pub1->publish(msg1);
      std::cout << "Publishing msg1: " << msg1.data << std::endl;
      pub3->publish(msg3);
      std::cout << "Publishing msg3: " << msg3.data << std::endl;
    });


  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  auto num_recv = std::size_t();
  while (num_recv < 3U) {
    // Waiting up to 5s for a sample to arrive.
    const auto wait_result = wait_set.wait(std::chrono::seconds(5));

    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      // Use sub1 as triggering condition for all subs?

      if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        bool message_received = sub1->take(msg, msg_info);
        if (message_received) {
          ++num_recv;
          std::cout << "msg1 data: " << msg.data << std::endl;
        }
      }

      if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        bool message_received = sub2->take(msg, msg_info);
        if (message_received) {
          ++num_recv;
          std::cout << "msg2 data: " << msg.data << std::endl;
        }
      }

      if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[2U]) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo msg_info;
        bool message_received = sub3->take(msg, msg_info);
        if (message_received) {
          ++num_recv;
          std::cout << "msg3 data: " << msg.data << std::endl;
        }
      }

      std::cout << "Number of messages already got: " << num_recv << " of 3" << std::endl;

    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      std::cerr << "No message received after 5s." << std::endl;
    } else {
      std::cerr << "Waitset failed" << std::endl;
    }
  }
  std::cout << "Got all messages!" << std::endl;

  thread.join();
  return 0;
}
